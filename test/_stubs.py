"""Minimal stand-ins for rospy, ROS messages, serial and gpiozero.

Lets the real node / driver code run off-robot (no ROS, no GPIO, no UART) so
the tests exercise the actual source files. Only what the code under test
touches is implemented.
"""

import os
import sys
import types

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, 'src')
TOOLS = os.path.join(REPO, 'etc', 'drift_tools')


# --------------------------------------------------------------------------- #
#  Generic message                                                            #
# --------------------------------------------------------------------------- #
class Msg:
    """Accepts any kwargs; missing attributes become nested Msg objects so
    `msg.header.stamp = ...` works like real ROS messages."""

    def __init__(self, *args, **kw):
        for i, v in enumerate(args):
            setattr(self, 'arg%d' % i, v)
        for k, v in kw.items():
            setattr(self, k, v)

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        v = Msg()
        object.__setattr__(self, name, v)
        return v


def _msg_type(name, fields=()):
    def init(self, *args, **kw):
        for f, v in zip(fields, args):
            kw.setdefault(f, v)
        for f in fields:
            kw.setdefault(f, 0.0)
        Msg.__init__(self, **kw)
    return type(name, (Msg,), {'__init__': init})


# --------------------------------------------------------------------------- #
#  rospy                                                                       #
# --------------------------------------------------------------------------- #
class Duration:
    def __init__(self, s):
        self.s = s

    def to_sec(self):
        return self.s


class Time:
    _now = 1000.0

    def __init__(self, s=0.0):
        self.s = float(s)

    @classmethod
    def now(cls):
        return cls(cls._now)

    def __sub__(self, other):
        return Duration(self.s - other.s)

    def to_sec(self):
        return self.s


class Publisher:
    instances = []

    def __init__(self, topic, mtype, **kw):
        self.topic, self.mtype, self.sent = topic, mtype, []
        Publisher.instances.append(self)

    def publish(self, msg):
        self.sent.append(msg)

    def get_num_connections(self):
        return 1


class Rate:
    def __init__(self, hz):
        pass

    def sleep(self):
        Time._now += 0.02


def make_rospy(params, shutdown_after=None):
    m = types.ModuleType('rospy')
    state = {'ticks': 0}

    def is_shutdown():
        if shutdown_after is None:
            return False
        state['ticks'] += 1
        return state['ticks'] > shutdown_after

    m.Publisher = Publisher
    m.Subscriber = lambda *a, **k: None
    m.Service = lambda *a, **k: None
    m.ServiceProxy = lambda *a, **k: (lambda *x, **y: None)
    m.get_param = lambda name, default=None: params.get(name, default)
    m.Time = Time
    m.Duration = Duration
    m.Rate = Rate
    m.is_shutdown = is_shutdown
    m.on_shutdown = lambda f: None
    m.sleep = lambda s: None
    m.logs = []
    for lvl in ('loginfo', 'logwarn', 'logerr'):
        setattr(m, lvl, (lambda lvl: lambda msg, *a: m.logs.append((lvl, msg)))(lvl))
    for lvl in ('logwarn_throttle', 'logerr_throttle', 'loginfo_throttle'):
        setattr(m, lvl, (lambda lvl: lambda p, msg, *a: m.logs.append((lvl, msg)))(lvl))
    m.init_node = lambda *a, **k: None
    m.spin = lambda: None
    return m


# --------------------------------------------------------------------------- #
#  serial / gpiozero                                                           #
# --------------------------------------------------------------------------- #
class FakeSerial:
    def __init__(self, port=None, baudrate=None, timeout=None, **kw):
        self.is_open = True
        self.written = bytearray()

    def open(self):
        self.is_open = True

    def write(self, data):
        self.written += bytes(data)
        return len(data)

    def read(self, size=1):
        return b'\x00' * size

    def close(self):
        self.is_open = False


class FakeEncoder:
    def __init__(self, a, b, max_steps=0, **kw):
        self.steps = 0


class FakeLED:
    def __init__(self, pin, **kw):
        self.state = False

    def on(self):
        self.state = True

    def off(self):
        self.state = False


def install(params=None, shutdown_after=None):
    """Install all stubs into sys.modules and put src/ and tools on sys.path."""
    for p in (SRC, TOOLS):
        if p not in sys.path:
            sys.path.insert(0, p)

    serial = types.ModuleType('serial')
    serial.Serial = FakeSerial
    sys.modules['serial'] = serial

    gz = types.ModuleType('gpiozero')
    gz.RotaryEncoder = FakeEncoder
    gz.LED = FakeLED

    class BadPinFactory(Exception):
        msg = 'bad pin factory'
    gz.BadPinFactory = BadPinFactory
    sys.modules['gpiozero'] = gz

    sys.modules['rospy'] = make_rospy(params or {}, shutdown_after)

    def pkg(name, **attrs):
        mod = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(mod, k, v)
        sys.modules[name] = mod
        return mod

    pkg('sensor_msgs')
    pkg('sensor_msgs.msg', LaserScan=_msg_type('LaserScan'), JointState=_msg_type('JointState'),
        Joy=_msg_type('Joy'))
    pkg('std_msgs')
    pkg('std_msgs.msg', Float32=_msg_type('Float32', ('data',)), Header=_msg_type('Header'),
        Int16MultiArray=_msg_type('Int16MultiArray'), String=_msg_type('String', ('data',)),
        Bool=_msg_type('Bool', ('data',)))
    pkg('geometry_msgs')
    pkg('geometry_msgs.msg', Twist=_msg_type('Twist'), TwistStamped=_msg_type('TwistStamped'),
        Vector3=_msg_type('Vector3', ('x', 'y', 'z')), Point=_msg_type('Point', ('x', 'y', 'z')))
    pkg('std_srvs')
    pkg('std_srvs.srv', SetBool=Msg, SetBoolRequest=Msg, SetBoolResponse=Msg,
        Trigger=Msg, TriggerRequest=Msg, TriggerResponse=Msg)
    pkg('foma')
    pkg('foma.srv', Float=Msg, FloatRequest=Msg, FloatResponse=Msg, String=Msg,
        StringRequest=Msg, StringResponse=Msg)

    class RPLidarException(Exception):
        pass

    class RPLidar:
        def __init__(self, *a, **k):
            raise RPLidarException('no sensor in tests')
    pkg('adafruit_rplidar', RPLidar=RPLidar, RPLidarException=RPLidarException)

    Publisher.instances = []
    return sys.modules['rospy']


def fresh_import(name):
    """Import (or re-import) a module after stubs are installed."""
    for k in list(sys.modules):
        if k == name or k.startswith(name + '.'):
            del sys.modules[k]
    return __import__(name, fromlist=['*'])
