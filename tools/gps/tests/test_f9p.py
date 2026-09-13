"""No real device access. Packet serialization and real parser, synthetic responses."""
import io
import struct
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from pyubx2 import UBXMessage, GET, UBXReader
from pyubx2.ubxtypes_configdb import UBX_CONFIG_DATABASE

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import configure_f9p_moving_base as configure
import check_f9p_heading as diagnostic
from f9p_common import REAR, FRONT, HeadingMonitor, RTCMCounter, UBXStream, heading_status


def relpos(length_cm=92, hp=0, carr=2, valid=1, moving=1, heading_valid=1, itow=1000):
    payload = bytearray(64)
    payload[0] = 1
    struct.pack_into('<I', payload, 4, itow)
    struct.pack_into('<ii', payload, 20, length_cm, 7300000)
    struct.pack_into('<b', payload, 35, hp)
    struct.pack_into('<I', payload, 52, 62000)
    flags = 3 | (valid << 2) | (carr << 3) | (moving << 5) | (heading_valid << 8)
    struct.pack_into('<I', payload, 60, flags)
    return UBXReader.parse(UBXMessage('NAV', 'NAV-RELPOSNED', GET, payload=bytes(payload)).serialize())


def rtcm(kind, crc=0, used=2, subtype=0):
    return UBXMessage('RXM', 'RXM-RTCM', GET,
                      payload=struct.pack('<BBHHH', 2, crc | (used << 1), subtype, 0, kind))


def test_exact_validated_settings():
    common = {'CFG_RATE_MEAS': 1000, 'CFG_RATE_NAV': 1,
              'CFG_SIGNAL_GLO_ENA': 0, 'CFG_SIGNAL_GLO_L1_ENA': 0, 'CFG_SIGNAL_GLO_L2_ENA': 0,
              'CFG_SIGNAL_BDS_ENA': 1, 'CFG_SIGNAL_BDS_B1_ENA': 1, 'CFG_SIGNAL_BDS_B2_ENA': 1}
    assert REAR == {**common, 'CFG_UART2_BAUDRATE': 38400, 'CFG_UART2OUTPROT_UBX': 0,
                    'CFG_UART2OUTPROT_NMEA': 0, 'CFG_UART2OUTPROT_RTCM3X': 1,
                    **{f'CFG_MSGOUT_RTCM_3X_TYPE{k}_UART2': 1 for k in ('4072_0', '1074', '1094', '1124')}}
    assert FRONT == {**common, 'CFG_UART1_BAUDRATE': 38400, 'CFG_UART1INPROT_RTCM3X': 1,
                     'CFG_NAVHPG_DGNSSMODE': 3, 'CFG_USBOUTPROT_UBX': 1,
                     'CFG_MSGOUT_UBX_NAV_RELPOSNED_USB': 1, 'CFG_MSGOUT_UBX_RXM_RTCM_USB': 1}


@pytest.mark.parametrize('settings', [REAR, FRONT])
@pytest.mark.parametrize('save,layers', [(False, 1), (True, 7)])
def test_wire_values_and_layers(settings, save, layers):
    raw = configure.config_packet(settings, save).serialize()
    assert raw[:4] == b'\xb5\x62\x06\x8a'
    assert raw[6:10] == bytes([0, layers, 0, 0])
    offset = 10
    for key, value in settings.items():
        kid, kind = UBX_CONFIG_DATABASE[key]
        assert int.from_bytes(raw[offset:offset+4], 'little') == kid
        size = int(kind[1:])
        assert int.from_bytes(raw[offset+4:offset+4+size], 'little') == value
        offset += 4 + size
    assert offset == len(raw)-2


def test_cli_default_and_same_device(monkeypatch):
    args = configure.parser().parse_args(['--rear-port', 'rear', '--front-port', 'front'])
    assert not args.save
    with pytest.raises(SystemExit):
        configure.parser().parse_args(['--rear-port', 'a', '--front-port', 'b', '--ram-only', '--save'])
    factory = MagicMock()
    monkeypatch.setattr(configure.serial, 'Serial', factory)
    with pytest.raises(ValueError, match='distinct'):
        configure.main(['--rear-port', '/tmp/gps', '--front-port', '/tmp/../tmp/gps'])
    factory.assert_not_called()


def test_identity_and_nak():
    stream = MagicMock()
    stream.exchange.return_value = SimpleNamespace(extension_01=b'MOD=ZED-F9P\x00')
    assert configure.identify(stream) == ['MOD=ZED-F9P']
    stream.exchange.return_value = SimpleNamespace(extension_01=b'MOD=ZED-F9R\x00')
    with pytest.raises(RuntimeError, match='identify'):
        configure.identify(stream)
    stream.exchange.return_value = SimpleNamespace(identity='ACK-NAK')
    with pytest.raises(RuntimeError, match='rejected'):
        configure.apply(stream, REAR, False)


def test_verify_only_and_missing_key():
    stream = MagicMock()
    stream.exchange.return_value = SimpleNamespace(**REAR)
    configure.verify(stream, REAR, 'flash')
    packet = stream.exchange.call_args.args[0]
    assert packet.identity == 'CFG-VALGET' and packet.layer == 2
    assert packet.serialize()[2:4] == b'\x06\x8b'
    stream.exchange.return_value = SimpleNamespace()
    with pytest.raises(RuntimeError, match='mismatch'):
        configure.verify(stream, REAR, 'ram')


@pytest.mark.parametrize('hp,expected', [(-50, .915), (0, .92), (50, .925)])
def test_heading_units(hp, expected):
    result = heading_status(relpos(hp=hp))
    assert result['relPosLength_m'] == pytest.approx(expected)
    assert result['heading_deg'] == pytest.approx(73)
    assert result['accHeading_deg'] == pytest.approx(.62)
    assert result['pass'] and result['valid_heading_deg'] == 73


@pytest.mark.parametrize('changes', [{'carr': 1}, {'carr': 0}, {'valid': 0}, {'moving': 0},
                                    {'heading_valid': 0}, {'length_cm': 100}])
def test_invalid_heading(changes):
    result = heading_status(relpos(**changes))
    assert not result['pass'] and result['valid_heading_deg'] is None


def test_freshness_and_duplicate_epoch():
    monitor = HeadingMonitor(.92, .05, 2.5)
    assert not monitor.snapshot(0)['pass']
    monitor.update(relpos(), 0)
    monitor.update(relpos(), 2)
    assert monitor.snapshot(2.49)['pass']
    assert not monitor.snapshot(2.5)['pass']
    monitor.update(relpos(itow=2000, carr=1), 3)
    assert monitor.snapshot(3)['valid_heading_deg'] is None


def test_rtcm_received_vs_used_and_crc():
    counter = RTCMCounter()
    for kind in (4072, 1074, 1094, 1124):
        counter.update(UBXReader.parse(rtcm(kind).serialize()))
        assert counter.counts[kind]['received'] == counter.counts[kind]['used'] == 1
    counter.update(rtcm(1074, used=1))
    counter.update(rtcm(1124, crc=1))
    assert counter.counts[1074]['not_used'] == 1
    assert counter.counts[1124]['received'] == 1
    assert counter.crc_failed == 1
    assert counter.subtypes_4072 == {0: 1}


def test_fragmentation_noise_checksum():
    raw = relpos().serialize()
    broken = raw[:-1] + bytes([raw[-1] ^ 1])
    chunks = iter([b'noise$NMEA\r\n'+broken+raw[:1], raw[1:10], raw[10:]])
    port = MagicMock()
    port.read.side_effect = lambda size: next(chunks, b'')
    stream = UBXStream(port)
    messages = stream.read() + stream.read() + stream.read()
    assert len(messages) == 1 and messages[0].identity == 'NAV-RELPOSNED'
    assert stream.errors == 1
    port.write.assert_not_called()


def test_exchange_timeout_and_ack_matching(monkeypatch):
    clock = iter([0, 0, 4])
    monkeypatch.setattr('f9p_common.time.monotonic', lambda: next(clock))
    port = MagicMock()
    port.write.side_effect = len
    port.read.return_value = b''
    with pytest.raises(TimeoutError):
        UBXStream(port).exchange(configure.config_packet(REAR), lambda msg: True)
    assert port.write.call_count == 1


def test_offline_cli_without_serial(tmp_path, monkeypatch, capsys):
    path = tmp_path / 'stream.ubx'
    path.write_bytes(relpos().serialize() + b''.join(rtcm(k).serialize() for k in (4072,1074,1094,1124)))
    factory = MagicMock(side_effect=AssertionError('Hardware must not open'))
    monkeypatch.setattr(diagnostic.serial, 'Serial', factory)
    assert diagnostic.run(['--input', str(path)]) == 0
    assert diagnostic.run(['--input', str(path)], rtcm=True) == 0
    path.write_bytes(relpos(carr=1).serialize())
    assert diagnostic.run(['--input', str(path)]) == 1
    path.write_bytes(b'')
    assert diagnostic.run(['--input', str(path)]) == 1
    factory.assert_not_called()


@pytest.mark.parametrize('save,layers', [(False, ['ram']), (True, ['ram', 'bbr', 'flash'])])
def test_ack_then_all_requested_layers(monkeypatch, save, layers):
    stream = MagicMock()
    stream.exchange.return_value = SimpleNamespace(identity='ACK-ACK')
    verification = MagicMock()
    monkeypatch.setattr(configure, 'verify', verification)
    configure.apply(stream, FRONT, save)
    assert [call.args[2] for call in verification.call_args_list] == layers
    accept = stream.exchange.call_args.args[1]
    assert not accept(SimpleNamespace(identity='ACK-ACK', clsID=6, msgID=1))
    assert accept(SimpleNamespace(identity='ACK-ACK', clsID=6, msgID=0x8A))


def test_verify_only_never_applies_settings(monkeypatch):
    factory = MagicMock()
    monkeypatch.setattr(configure.serial, 'Serial', factory)
    identify = MagicMock(return_value=['MOD=ZED-F9P'])
    verify = MagicMock()
    apply = MagicMock(side_effect=AssertionError('VALSET forbidden'))
    monkeypatch.setattr(configure, 'identify', identify)
    monkeypatch.setattr(configure, 'verify', verify)
    monkeypatch.setattr(configure, 'apply', apply)
    assert configure.main(['--rear-port', '/tmp/rear', '--front-port', '/tmp/front',
                           '--verify-only', '--layer', 'flash']) == 0
    assert identify.call_count == 2 and verify.call_count == 2
    assert all(call.args[2] == 'flash' for call in verify.call_args_list)
    apply.assert_not_called()


def test_both_identified_before_any_write(monkeypatch):
    monkeypatch.setattr(configure.serial, 'Serial', MagicMock())
    monkeypatch.setattr(configure, 'identify', MagicMock(side_effect=[['MOD=ZED-F9P'], RuntimeError('wrong model')]))
    apply = MagicMock()
    monkeypatch.setattr(configure, 'apply', apply)
    with pytest.raises(RuntimeError, match='wrong model'):
        configure.main(['--rear-port', '/tmp/rear', '--front-port', '/tmp/front', '--save'])
    apply.assert_not_called()
