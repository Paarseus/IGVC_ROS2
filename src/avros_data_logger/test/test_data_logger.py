import csv
from threading import Lock

import cv2
from sensor_msgs.msg import Imu, NavSatFix

from avros_data_logger.data_logger_node import (
    CAMERA_COLUMNS,
    GNSS_COLUMNS,
    IMU_COLUMNS,
    _image_names,
    _open_camera,
    DataLoggerNode,
)


def test_camera_opening_uses_requested_index(monkeypatch):
    calls = []

    class FakeCamera:
        def isOpened(self):
            return True

        def release(self):
            pass

    def fake_capture(index):
        calls.append(index)
        return FakeCamera()

    monkeypatch.setattr(cv2, 'VideoCapture', fake_capture)
    camera = _open_camera(7)

    assert calls == [7]
    assert camera.isOpened()


def test_camera_opening_fails_for_unavailable_device(monkeypatch):
    class FakeCamera:
        def isOpened(self):
            return False

        def release(self):
            self.released = True

    monkeypatch.setattr(cv2, 'VideoCapture', lambda index: FakeCamera())

    try:
        _open_camera(4)
    except RuntimeError as exc:
        assert 'camera index 4' in str(exc)
    else:
        raise AssertionError('Unavailable camera should raise RuntimeError')


def test_image_names_are_matching_and_zero_padded():
    names = _image_names(12, '20261001T143015.123456Z')

    assert names == (
        'rgb_000012_20261001T143015.123456Z.jpg',
        'thermal_000012_20261001T143015.123456Z.jpg',
    )


def test_csv_headers_are_distinct_and_complete(tmp_path):
    camera_path = tmp_path / 'camera_log.csv'
    imu_path = tmp_path / 'imu_log.csv'
    gnss_path = tmp_path / 'gnss_log.csv'
    files = []
    for path, columns in ((camera_path, CAMERA_COLUMNS),
                          (imu_path, IMU_COLUMNS),
                          (gnss_path, GNSS_COLUMNS)):
        file, writer = DataLoggerNode._open_csv(path, columns)
        files.append(file)
        writer.writerow({column: '' for column in columns})
        file.flush()

    for file in files:
        file.close()

    with camera_path.open(newline='') as file:
        assert next(csv.reader(file)) == CAMERA_COLUMNS
    with imu_path.open(newline='') as file:
        assert next(csv.reader(file)) == IMU_COLUMNS
    with gnss_path.open(newline='') as file:
        assert next(csv.reader(file)) == GNSS_COLUMNS
    assert {'image_count', 'episode_id', 'rgb_image', 'thermal_image', 'command_steering'}.issubset(CAMERA_COLUMNS)
    assert {'orientation_x', 'angular_velocity_z', 'linear_acceleration_x'}.issubset(IMU_COLUMNS)
    assert {'latitude', 'longitude', 'position_covariance_0'}.issubset(GNSS_COLUMNS)


def test_every_imu_message_is_written(tmp_path):
    path = tmp_path / 'imu_log.csv'
    file, writer = DataLoggerNode._open_csv(path, IMU_COLUMNS)
    node = DataLoggerNode.__new__(DataLoggerNode)
    node._imu_file = file
    node._imu_writer = writer
    node._lock = Lock()

    message = Imu()
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 34
    message.angular_velocity.z = 1.25
    node._on_imu(message)
    file.close()

    with path.open(newline='') as saved:
        rows = list(csv.DictReader(saved))
    assert len(rows) == 1
    assert rows[0]['timestamp_ns'] == '12000000034'
    assert rows[0]['angular_velocity_z'] == '1.25'


def test_every_gnss_message_is_written(tmp_path):
    path = tmp_path / 'gnss_log.csv'
    file, writer = DataLoggerNode._open_csv(path, GNSS_COLUMNS)
    node = DataLoggerNode.__new__(DataLoggerNode)
    node._gnss_file = file
    node._gnss_writer = writer
    node._lock = Lock()

    message = NavSatFix()
    message.header.stamp.sec = 8
    message.latitude = 34.0
    message.longitude = -117.0
    message.position_covariance[0] = 0.5
    node._on_gnss(message)
    file.close()

    with path.open(newline='') as saved:
        rows = list(csv.DictReader(saved))
    assert len(rows) == 1
    assert rows[0]['timestamp_ns'] == '8000000000'
    assert rows[0]['latitude'] == '34.0'
    assert rows[0]['position_covariance_0'] == '0.5'
