import functools
import random
import sys


def deterministic_bytes_examples(max_examples, seed):
  """Generate deterministic arbitrary byte-list examples for parser smoke tests."""
  edge_cases = (
    [],
    [b""],
    [b"\x00"],
    [b"\xff"],
    [b"\x00", b"\xff", b"\x00\xff"],
    [bytes(range(256))],
    [b"\x00" * 64],
    [b"\xff" * 64],
    [b"\xf1\x00", b"\x02" + (b"\x00" * 31)],
  )

  for i in range(max_examples):
    if i < len(edge_cases):
      yield edge_cases[i]
      continue

    rng = random.Random(f"{seed}-{i}")
    list_len = rng.randrange(0, 12)
    if i % 17 == 0:
      list_len += rng.randrange(12, 32)

    yield [rng.randbytes(rng.randrange(0, 96)) for _ in range(list_len)]


def parameterized(argnames, argvalues):
  """Method decorator that runs a test once per parameter set using subTest.

  Usage:
    @parameterized("x, y", [(1, 2), (3, 4)])
    def test_add(self, x, y): ...

    @parameterized("car_model, fingerprints", FINGERPRINTS.items())
    def test_fw(self, car_model, fingerprints): ...
  """
  if isinstance(argnames, str):
    argnames = [a.strip() for a in argnames.split(',')]

  def decorator(func):

    @functools.wraps(func)
    def wrapper(self):
      for values in argvalues:
        if not isinstance(values, (tuple, list)):
          values = (values,)
        kwargs = dict(zip(argnames, values, strict=True))
        with self.subTest(**kwargs):
          func(self, **kwargs)

    return wrapper

  return decorator


def parameterized_class(attrs, values=None):
  """Class decorator that generates subclasses with different class attributes.

  Usage:
    @parameterized_class([{"x": 1}, {"x": 2}])
    @parameterized_class('x', [(1,), (2,)])
  """
  if isinstance(attrs, str):
    attrs = [attrs]
    params = [dict(zip(attrs, v, strict=True)) for v in values]
  else:
    params = attrs

  def decorator(cls):
    module = sys.modules[cls.__module__]
    for param_set in params:
      name = f"{cls.__name__}_{'_'.join(str(v) for v in param_set.values())}"
      new_cls = type(name, (cls,), param_set)
      new_cls.__qualname__ = name
      new_cls.__module__ = cls.__module__
      new_cls.__test__ = True
      setattr(module, name, new_cls)
    cls.__test__ = False
    return cls

  return decorator
