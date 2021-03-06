"""
Documentation testing

Based on:
https://github.com/cprogrammer1994/ModernGL/blob/master/tests/test_documentation.py
by Szabolcs Dombi
"""
import inspect
import os
import re
import unittest

os.environ['DEMOSYS_SETTINGS_MODULE'] = 'tests.settings'  # noqa

import demosys
from demosys import effects, opengl

# Modules we want to remove from types
MODULES = [
    'demosys.',
    'opengl.',
    'texture.',
    'shader.',
    'resources.',
    'data.',
    'moderngl.',
    'buffer.',
    'rocket.',
    'tracks.',
    'scene.',
    'buffer.',
    'depth.',
    'texture_array.',
    'program.',
    'vertex_array.',
    'array.',
]

class TestCase(unittest.TestCase):

    def validate(self, filename, module, classname, ignore):
        root = os.path.dirname(os.path.dirname(__file__))
        with open(os.path.normpath(os.path.join(root, 'docs', 'source', filename))) as f:
            docs = f.read()
        methods = re.findall(r'^\.\. automethod:: ([^\(\n]+)([^\n]+)', docs, flags=re.M)
        attributes = re.findall(r'^\.\. autoattribute:: ([^\n]+)', docs, flags=re.M)
        documented = set(filter(lambda x: x.startswith(classname), [a for a, b in methods] + attributes))
        implemented = set(classname + '.' + x for x in dir(getattr(module, classname)) if not x.startswith('_'))
        ignored = set(classname + '.' + x for x in ignore)
        self.assertSetEqual(implemented - documented - ignored, set(), msg='Implemented but not Documented')
        self.assertSetEqual(documented - implemented, set(), msg='Documented but not Implemented')

        for method, docsig in methods:
            classname, methodname = method.split('.')
            sig = str(inspect.signature(getattr(getattr(module, classname), methodname)))
            print(sig)
            sig = sig.replace('self, ', '').replace('typing.', '').replace(' -> None', '')

            for m in MODULES:
                sig = sig.replace(m, '')

            sig = sig.replace('(self)', '()').replace(', *,', ',').replace('(*, ', '(')
            sig = re.sub(r'-> \'(\w+)\'', r'-> \1', sig)

            self.assertEqual(docsig, sig, msg=filename + '::' + method)

    # def test_effect_docs(self):
    #     self.validate(
    #         os.path.join('reference', 'effect.rst'),
    #         effects, 'Effect', [])

    # def test_vao_docs(self):
    #     self.validate(
    #         os.path.join('reference', 'vao.rst'),
    #         opengl, 'VAO', []
    #     )

if __name__ == '__main__':
    unittest.main()
