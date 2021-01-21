import sys
import inspect
import types


def get_size(obj, seen=None):
    """
    Gets the size of the given object. Supports integer, string, float, dict, list, set, tuple and user
    defined classes. Supports nasted mutable data structures.
    :param obj:
    :param seen: Objects seen so far. Identity verified by id(obj)
    :return: size of object.
    """
    size = 0
    uid = id(obj)
    if seen is None:
        seen = set()

    if uid in seen:
        return 0

    seen.add(uid)

    if obj is None or obj.__class__ in (int, float, bool, complex, str, bytes, type):
        return sys.getsizeof(obj)

    elif obj.__class__ == bytearray:
        size += len(obj) + sys.getsizeof(obj.__class__())

    elif obj.__class__ in (list, tuple, set, frozenset):
        size += len(obj) * 8
        size += sys.getsizeof(obj.__class__())

        for elem in obj:
                size += get_size(elem, seen)

    elif obj.__class__ == dict:
        size += sys.getsizeof({})

        for elem in obj:
            size += 8
            size += get_size(elem, seen)
            size += get_size(obj[elem], seen)

    elif isinstance(obj, (range, memoryview, map, filter, types.GeneratorType, types.AsyncGeneratorType)):
        return sys.getsizeof(obj)

    else:
        for at in inspect.getmembers(obj, lambda at:not(inspect.isroutine(at))):
            if not callable(at[1]):
                size += 8
                size += get_size(at[1], seen)

    return size


if __name__ == "__main__":
    pass