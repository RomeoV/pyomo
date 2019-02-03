# coding: utf-8
from pyomo.environ import (ConcreteModel, Var, Constraint)
class Node:
    def __init__(self):
        self.commutative = None

    def evaluate(self, value_dict):
        raise NotImplementedError()

    def print(self):
        raise NotImplementedError()
        
class BinaryNode(Node):
    def __init__(self):
        self.commutative = False
        self.childL = None
        self.childR = None
        
class UnaryNode(Node):
    def __init__(self):
        self.commutative=False
        self.child = None

class LeafNode(UnaryNode):
    def __init__(self, single_node):
        self.child = single_node
    def print(self):
        if isinstance(self.child, str):
            return self.child
        elif isinstance(self.child, Var):
            return self.child.name
    def evaluate(self,value_dict):
        return value_dict[self.child]

class NotNode(UnaryNode):
    def __init__(self, single_node):
        self.child = single_node
    def print(self):
        output = self.child.print()
        return '!['+output+']'
    def evaluate(self,value_dict):
        return not self.child.evaluate(value_dict)
        
class MultiNode(Node):
    def __init__(self):
        self.commutative = True
        self.children = []
    def tryPurgingWithSingleChild(self):
        if len(self.children) == 1:
            child = self.children.pop()
            self.__class__ = type(child)
            self.__init__(child.children)
    def tryPurgingSameTypeChildren(self):
        if all([isinstance(n, type(self)) for n in self.children]):
            self.children = set.union(*[c.children for c in self.children])

        
class AndNode(MultiNode):
    def __init__(self, var_list):
        self.children = set([v for v in var_list])

    def print(self):
        output = " ^ ".join([c.print() for c in self.children])
        return '{'+output+'}'

    def evaluate(self, value_dict):
        return all(n.evaluate(value_dict) for n in self.children)

    def distributivity_or_in_and(self):
        self.tryPurgingWithSingleChild()
        self.tryPurgingSameTypeChildren()
        for n in [c for c in self.children if isinstance(c, OrNode)]:
            n.distributivity_or_in_and()
        self.tryPurgingWithSingleChild()
        self.tryPurgingSameTypeChildren()

    def distributivity_and_in_or(self):
        while any([isinstance(n, OrNode) for n in self.children]) and len(self.children) > 1:
            or_node = next(n for n in self.children if isinstance(n, OrNode))
            other_nodes = set([n for n in self.children if not n is or_node])
            new_or_node = OrNode([AndNode(set([or_el]) | other_nodes) for or_el in or_node.children])
            self.children -= set([or_node]) | other_nodes
            self.children |= set([new_or_node])
        self.tryPurgingWithSingleChild()
        self.tryPurgingSameTypeChildren()
        for n in filter(isAndNode, self.children):
            n.distributivity_and_in_or()
        self.tryPurgingWithSingleChild()
        self.tryPurgingSameTypeChildren()
        
class OrNode(MultiNode):
    def __init__(self,var_list):
        self.children = set([v for v in var_list])

    def print(self):
        output = " v ".join([c.print() for c in self.children])
        return '('+output+')'

    def evaluate(self, value_dict):
        return any([n.evaluate(value_dict) for n in self.children])

    def distributivity_or_in_and(self):
        while any([isinstance(n, AndNode) for n in self.children]) and len(self.children) > 1:
            and_node = next(n for n in self.children if isinstance(n, AndNode))
            other_nodes = set([n for n in self.children if not n is and_node])
            new_and_node = AndNode([OrNode(set([and_el]) | other_nodes) for and_el in and_node.children])
            self.children -= set([and_node]) | other_nodes
            self.children |= set([new_and_node])

        self.tryPurgingWithSingleChild()
        self.tryPurgingSameTypeChildren()
        for n in filter(isAndNode, self.children):
            n.distributivity_or_in_and()
        self.tryPurgingWithSingleChild()
        self.tryPurgingSameTypeChildren()

    def distributivity_and_in_or(self):
        self.tryPurgingWithSingleChild()
        self.tryPurgingSameTypeChildren()
        for n in filter(isOrNode, self.children):
            n.distributivity_and_in_or()
        self.tryPurgingWithSingleChild()
        self.tryPurgingSameTypeChildren()

isOrNode = lambda n: isinstance(n, OrNode)
isNotOrNode = lambda n: not isinstance(n, OrNode)
isAndNode = lambda n: isinstance(n, AndNode)
isNotAndNode = lambda n: not isinstance(n, AndNode)
isNode = lambda n: isinstance(n, Node)
isLeafNode = lambda n: not isinstance(n, LeafNode)