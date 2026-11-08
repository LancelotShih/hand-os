"""Every setting in one place.  Adjust freely."""

from pathlib import Path

HERE = Path(__file__).resolve().parent

# hardware
I2C_BUS = 1
I2C_ADDRESS = 0x68
MMC_I2C_ADDRESS = 0x30
ICM_USE_CALIBRATION = True    # start with the tumble's midpoints and corrections applied?
MMC_USE_CALIBRATION = False   # no: SET/RESET already removes the sensor's own offset
MMC_BANDWIDTH_HZ = 800    # shortest conversion, 0.5 ms; slower ones are quieter but smear a turn

# web view
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8000
PAGE_DIR = HERE / "page"

# orientation
AHRS_BETA = 0.05          # Madgwick gain: accelerometer's pull on tilt
MAG_LATENCY_S = 0.012     # AK09916 sample age when read
HISTORY_S = 0.25          # orientation history kept to line up with that age
MAX_GAP_S = 0.2           # a longer pause in the sensor loop can't be integrated

# capturing a reading
CAPTURE_S = 0.3           # time averaged per capture ...
CAPTURE_MIN_MAG = 20      # ... with at least this many magnetometer samples
CAPTURE_MAX_TURN_DEG = 15.0   # only turning further than this restarts a capture

# gyro bias
REST_S = 1.0              # resting this long measures the gyro bias:
REST_SPREAD_DPS = 2.0     # no gyro axis swings more than this ...
REST_MAX_BIAS_DPS = 3.0   # ... or averages more than this (that's a slow turn)

# judging the result
RATE_WINDOW_S = 1.0       # each magnetometer's readings per second is counted over this long
BLOCK_SAMPLES = 5         # magnetometer samples per view update (100 Hz -> 20 Hz)
MMC_BLOCK_SAMPLES = 3     # the same for the MMC5983MA: about the same 50 ms, so the two
                          # are averaged over equal time and neither gets an easier test
PASS_UT = 5.0             # the field accuracy we want: green under this, everywhere
PASS_MIN_TURN_DEG = 30.0  # a error only proves much once the board has turned this far

# tumble
TUMBLE_MIN_POINTS = 500   # magnetometer samples needed, about 6 s
TUMBLE_MAX_POINTS = 12000 # about 2 minutes; past this the readings are thinned
TUMBLE_MIN_COVERAGE = 0.75    # share of directions reached before Finish is allowed
TUMBLE_COVERAGE_RECOUNT = 25  # samples between coverage updates
TUMBLE_MAX_GAIN = 1.4     # a stretch beyond this isn't the sensor
TUMBLE_MIN_SWING_UT = 20.0    # every direction must swing at least this far either way;
                              # Earth's field is 25-65 uT, so noise on a still board can't
TUMBLE_MIN_SWING_RATIO = 0.8  # ... and the smallest swing at least this share of the largest:
                              # a partial turn looks centred, but its swings don't match
TUMBLE_MAX_FIELD_UT = 100.0   # a field stronger than this isn't Earth's

# files
CALIBRATION_FILE = HERE / "earth_cancel_offset.json"
TUMBLE_LOG_FILE = HERE / "earth_cancel_tumble_log.json"   # last tumble's readings and result
MMC_CALIBRATION_FILE = HERE / "earth_cancel_mmc_offset.json"
MMC_TUMBLE_LOG_FILE = HERE / "earth_cancel_mmc_tumble_log.json"
MMC_MOUNTING_FILE = HERE / "mmc_mounting.json"   # how the chip sits on the board; a fact about
                                                 # the hardware, so Reset calibration keeps it
LOG_DIR = HERE / "logs"   # each log gets its own folder in here, named by its start time
