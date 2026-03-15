from enum import Enum, auto
from typing import Literal, Tuple, Union

import carla
import numpy as np
from PIL import Image

from ..karma_data_provider import KarmaDataProvider


class CaptureFailureMode(Enum):
    none = auto()
    zero = auto()
    noise = auto()
    last = auto()


class Camera(object):
    def __init__(self,
                 sensor: carla.Sensor,
                 capture_failure_mode: CaptureFailureMode = CaptureFailureMode.zero,
                 register: bool = True,
                 **kwargs
                 ):
        if not sensor.type_id.startswith('sensor.camera'):
            raise ValueError('Sensor must be a camera.')

        self._camera = sensor

        # TODO: add support for other camera types
        self._type = self._camera.type_id.replace('sensor.camera.', '')
        self._converter = {
            'rgb': carla.ColorConverter.Raw,
            'depth': carla.ColorConverter.LogarithmicDepth if kwargs.get('logarithmic', False) else carla.ColorConverter.Depth,
            'semantic_segmentation': carla.ColorConverter.CityScapesPalette,
            'instance_segmentation': carla.ColorConverter.CityScapesPalette,
            'dvs': carla.ColorConverter.Raw,
            'optical_flow': carla.ColorConverter.Raw,
        }[self._type]

        self._image_size = (
            int(self._camera.attributes['image_size_x']),
            int(self._camera.attributes['image_size_y'])
        )
        self._image_shape = (self._image_size[1], self._image_size[0], 3)

        self._capture_failure_mode = capture_failure_mode
        self._last_frame = None
        if self._capture_failure_mode == 'last':
            self._last_frame = np.zeros(self._image_shape, dtype=np.uint8)

        if register:
            KarmaDataProvider.register_sensor_queue(self._camera)

    @property
    def image_size(self) -> Tuple[int]:
        return self._image_size

    @property
    def sensor(self) -> carla.Sensor:
        return self._camera

    @property
    def camera_type(self) -> str:
        return self._type

    def get_transform(self) -> carla.Transform:
        return self._camera.get_transform()

    def handle_capture_failure(self) -> Union[np.ndarray, None]:
        if self._capture_failure_mode == CaptureFailureMode.zero:
            self._last_frame = np.zeros(self._image_shape, dtype=np.uint8)
        elif self._capture_failure_mode == CaptureFailureMode.noise:
            self._last_frame = KarmaDataProvider.get_rng().randint(
                0, 256, size=self._image_shape, dtype=np.uint8)
        elif self._capture_failure_mode == CaptureFailureMode.last:
            self._last_frame = self._last_frame.copy()
        else:
            self._last_frame = None

        return self._last_frame.copy()

    def get_rgb_data(self) -> Union[np.ndarray, None]:
        """
        Returns the most recent image from camera as an RGB numpy array (PIL-compatible).
        Depending on settings, can return None, zeros array, random noise or repeated last frame
        if no image is available.

        :return: [description]
        :rtype: Union[np.ndarray, None]
        """
        frames = KarmaDataProvider.get_sensor_data(self._camera.id)

        if len(frames):
            data = frames[-1]

            if self._converter != carla.ColorConverter.Raw:
                # convert to human- & mp4-friendly format
                data.convert(self._converter)

            img = Image.frombuffer('RGBA', (data.width, data.height),
                                   data.raw_data, 'raw', 'RGBA', 0, 1)  # load

            img = img.convert('RGB')  # drop alpha
            # the data is actually in BGR format, so switch channels
            self._last_frame = np.array(img)[..., ::-1]

            return self._last_frame.copy()

        return self.handle_capture_failure()

    def get_segmentation_data(self) -> Union[np.ndarray, None]:
        """
        Returns the most recent image from camera as an RGB numpy array (PIL-compatible).
        Depending on settings, can return None, zeros array, random noise or repeated last frame
        if no image is available.

        The segmentation data encoded in the red channel. Green & blue are used for optional instance IDs.

        :return: [description]
        :rtype: Union[np.ndarray, None]
        """
        frames = KarmaDataProvider.get_sensor_data(self._camera.id)

        if len(frames):
            data = frames[-1]

            self._last_frame = np.frombuffer(data.raw_data, dtype=np.uint8).reshape(
                (data.height, data.width, -1))[..., :3][..., ::-1]

            return self._last_frame.copy()

        return self.handle_capture_failure()

    def get_dvs_data(self) -> Union[np.ndarray, None]:
        """
        Returns the events stream batched by frame.
        Depending on settings, can return None, zeros array, random noise or repeated last frame
        if no data is available.

        The DVS data encodes 0 as no change, 1 for negative polarity and 2 for positive polarity.

        :return: [description]
        :rtype: Union[np.ndarray, None]
        """
        frames = KarmaDataProvider.get_sensor_data(self._camera.id)
        event_dtype = np.dtype([
            ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', bool)])

        if len(frames):
            data = frames[-1]
            self._last_frame = np.frombuffer(data.raw_data, dtype=event_dtype)

            return self._last_frame.copy()

        # special failure handling for array mode
        if self._capture_failure_mode == CaptureFailureMode.zero:
            self._last_frame = np.empty((0,), dtype=event_dtype)
        elif self._capture_failure_mode == CaptureFailureMode.noise:
            prev_min_t = self._last_frame['t'].min() if len(self._last_frame) else 0
            prev_max_t = self._last_frame['t'].max() if len(
                self._last_frame) else 33  # arbitrary, assuming 30 fps

            # randomly change ~5% of the pixels
            no_of_events = max(0,
                                int(KarmaDataProvider.get_rng().normal(
                                    0.05 * self._image_size[1] * self._image_size[0],
                                    0.01 * self._image_size[1] * self._image_size[0]
                                )))

            self._last_frame = np.empty((no_of_events,), dtype=event_dtype)
            self._last_frame['x'] = KarmaDataProvider.get_rng().randint(
                0, self._image_size[0], size=no_of_events, dtype=np.uint16)
            self._last_frame['y'] = KarmaDataProvider.get_rng().randint(
                0, self._image_size[1], size=no_of_events, dtype=np.uint16)
            self._last_frame['pol'] = KarmaDataProvider.get_rng().randint(
                0, 2, size=no_of_events, dtype=bool)

            # TODO: make the timestamp more realistic
            self._last_frame['t'] = KarmaDataProvider.get_rng().randint(
                0, prev_max_t-prev_min_t, size=no_of_events, dtype=np.int64) + prev_max_t
        elif self._capture_failure_mode == CaptureFailureMode.last:
            self._last_frame = self._last_frame.copy()
        else:
            self._last_frame = None

        return self._last_frame.copy()
