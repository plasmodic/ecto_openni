#!/usr/bin/env python

import ecto_openni
import unittest

class TestOpenni(unittest.TestCase):

    def test_import(self):
        print ecto_openni

if __name__ == '__main__':
    import rostest
    rostest.rosrun('ecto_openni', 'test_openni', TestOpenni)
