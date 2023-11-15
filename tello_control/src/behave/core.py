# coding=utf-8
from functools import partial

##############################################################################
SUCCESS = "Success"
FAILURE = "Failure"
RUNNING = "Running"


##############################################################################
class BehaveException(Exception):
    pass

def wrap_iterator(iterator):
    def wrapped():
        if wrapped.done:
            raise BehaveException("Ticking a finished node.")
        x = iterator()
        if x != RUNNING:
            wrapped.done = True

    wrapped.done = False
    return wrapped

##############################################################################
class Blackboard(object):
    def __init__(self, node, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.tick = self.new_iterator(node)

    def new_iterator(self, node):
        it = node.Iterator(self, node)
        it.done = False
        def it_func():
            if it.done:
                raise BehaveException("Ticking a finished node.")
            x = it()
            if x != RUNNING:
                it.done = True
            return x

        return it_func


class DebugBlackboard(Blackboard):
    def __init__(self, node, debugger, *args, **kwargs):
        self.debugger = debugger
        super(DebugBlackboard, self).__init__(node, *args, **kwargs)

    def new_iterator(self, node):
        it = Blackboard.new_iterator(self, node)
        def wrapper():
            x = it()
            self.debugger(node, x)
            return x
        return wrapper


##############################################################################
class BeNode(object):
    def __init__(self):
        self.desc = None
        self._name = None

    @property
    def name(self):
        return self._name or self.__class__.__name__

    @name.setter
    def name(self, value):
        self._name = value

    def blackboard(self, *args, **kwargs):
        return Blackboard(self, *args, **kwargs)

    def debug(self, debugger, *args, **kwargs):
        return DebugBlackboard(self, debugger, *args, **kwargs)

    def clone(self):
        c = self.__class__()
        c.copy_from(self)
        return c

    def copy_from(self, other):
        self.desc = other.desc

    def __or__(self, sibling):
        return BeSelect([self, sibling])

    def __rshift__(self, sibling):
        return BeSequence([self, sibling])

    def __floordiv__(self, desc):
        c = self.clone()
        c.desc = desc
        return c


##############################################################################
class BeAction(BeNode):
    def __init__(self, func=None):
        super(BeAction, self).__init__()
        self.func = func

    @BeNode.name.getter
    def name(self):
        if self._name:
            return self._name
        func = self.func
        if func and hasattr(func, "__name__"):
            return func.__name__
        else:
            return self.__class__.__name__

    def copy_from(self, other):
        super(BeAction, self).copy_from(other)
        self.func = other.func

    class Iterator(object):
        def __init__(self, bb, node):
            self.func = partial(node.func, *bb.args, **bb.kwargs)

        def __call__(self):
            x = self.func()
            if x is None:
                return SUCCESS
            assert x == SUCCESS or x == FAILURE or x == RUNNING
            return x

##############################################################################
class BeGeneratorAction(BeNode):
    def __init__(self, generatorfunc=None):
        super(BeGeneratorAction, self).__init__()
        self.generatorfunc = generatorfunc

    @BeNode.name.getter
    def name(self):
        if self._name:
            return self._name
        func = self.generatorfunc
        if func and hasattr(func, "__name__"):
            return func.__name__
        else:
            return self.__class__.__name__

    def copy_from(self, other):
        super(BeGeneratorAction, self).copy_from(other)
        self.generatorfunc = other.generatorfunc

    class Iterator(object):
        def __init__(self, bb, node):
            self.generator = node.generatorfunc(*bb.args, **bb.kwargs)

        def __call__(self):
            try:
                x = next(self.generator)
                if x is None:
                    return RUNNING
                assert x == SUCCESS or x == FAILURE or x == RUNNING
                return x
            except StopIteration:
                return SUCCESS


##############################################################################
class BeCondition(BeNode):
    def __init__(self, func=None):
        super(BeCondition, self).__init__()
        self.func = func

    @BeNode.name.getter
    def name(self):
        if self._name:
            return self._name
        func = self.func
        if func and hasattr(func, "__name__"):
            return func.__name__
        else:
            return self.__class__.__name__

    def copy_from(self, other):
        super(BeCondition, self).copy_from(other)
        self.func = other.func

    class Iterator(object):
        def __init__(self, bb, node):
            self.func = partial(node.func, *bb.args, **bb.kwargs)

        def __call__(self):
            return SUCCESS if self.func() else FAILURE


##############################################################################
class BeComposite(BeNode):
    def __init__(self, children=None):
        super(BeComposite, self).__init__()
        self.children = children or []

    def copy_from(self, other):
        super(BeComposite, self).copy_from(other)
        self.children = other.children[:]


##############################################################################
class BeSelect(BeComposite):
    def __or__(self, child):
        children = self.children[:]
        children.append(child)
        return BeSelect(children)

    class Iterator(object):
        def __init__(self, bb, node):
            self.iterations = self._make_iterations(bb, node)

        def _make_iterations(self, bb, node):
            for c in node.children:
                it = bb.new_iterator(c)
                while True:
                    x = it()
                    if x == RUNNING:
                        yield x
                    elif x == SUCCESS:
                        yield x
                        return
                    else:
                        assert x == FAILURE
                        break

            # all children are failed
            yield FAILURE

        def __call__(self):
            return next(self.iterations)


##############################################################################
class BeSequence(BeComposite):
    def __rshift__(self, child):
        children = self.children[:]
        children.append(child)
        return BeSequence(children)

    class Iterator(object):
        def __init__(self, bb, node, *args, **kwargs):
            self.iterations = self._make_iterations(bb, node)

        def _make_iterations(self, bb, node):
            for c in node.children:
                it = bb.new_iterator(c)
                while True:
                    x = it()
                    if x == RUNNING:
                        yield x
                    elif x == FAILURE:
                        yield x
                        return
                    else:
                        assert x == SUCCESS
                        break

            # all children are failed
            yield SUCCESS

        def __call__(self):
            return next(self.iterations)


##############################################################################
class BeDecorator(object):
    def __init__(self, decorators=None):
        self.decorators = decorators or []

    def __call__(self, node):
        assert isinstance(node, BeNode)
        return BeDecorated(self.decorator, node)

    def __mul__(self, other):
        if isinstance(other, BeDecorator):
            decorators = self.decorators[:]
            decorators.extend(other.decorators)
            return BeDecorator(decorators)
        elif isinstance(other, BeNode):
            node = other
            for deco in reversed(self.decorators):
                node = BeDecorated(deco, node)
            return node

        raise TypeError("Decorator should connect to another decorator or node")


##############################################################################
class BeDecorated(BeNode):
    def __init__(self, decorator=None, node=None):
        super(BeDecorated, self).__init__()
        self.decorator = decorator
        self.node = node

    @BeNode.name.getter
    def name(self):
        if self._name:
            return self._name
        decorator = self.decorator
        node = self.node
        if node and decorator and hasattr(decorator, "__name__"):
            return decorator.__name__ + "*" + node.name
        else:
            return self.__class__.__name__

    def copy_from(self, other):
        super(BeDecorated, self).copy_from(other)
        self.decorator = other.decorator
        self.node = other.node

    class Iterator(object):
        def __init__(self, bb, node):
            self.decorator_instance = node.decorator(bb, node.node)

        def __call__(self):
            x = self.decorator_instance()
            if x is None:
                return SUCCESS
            assert x == SUCCESS or x == FAILURE or x == RUNNING
            return x

