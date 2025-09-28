import unittest

class TestDesiredDirection(unittest.TestCase):  # Fixed the class inheritance
    def setUp(self):
        # Create a mock object to simulate the class containing the method
        class MockDetector:
            def is_pair_in_desired_direction(self, angle1, angle2, desired_directions, tolerance=15.0):
                a1 = angle1 % 360
                a2 = angle2 % 360
                for dir1, dir2 in desired_directions:
                    if (abs(a1 - dir1) <= tolerance and abs(a2 - dir2) <= tolerance) or \
                       (abs(a2 - dir1) <= tolerance and abs(a1 - dir2) <= tolerance):
                        return True
                return False

        self.detector = MockDetector()

    def test_case_1(self):
        # Test NW-SE direction (135, 315)
        self.assertTrue(self.detector.is_pair_in_desired_direction(135, 315, [(135, 315)], 15))

    def test_case_2(self):
        # Test NE-SW direction (45, 225)
        self.assertTrue(self.detector.is_pair_in_desired_direction(45, 225, [(45, 225)], 15))

    def test_case_3(self):
        # Test angles outside tolerance
        self.assertFalse(self.detector.is_pair_in_desired_direction(150, 330, [(135, 315)], 10))

    def test_case_4(self):
        # Test angles with reversed order
        self.assertTrue(self.detector.is_pair_in_desired_direction(315, 135, [(135, 315)], 15))

    def test_case_5(self):
        # Test multiple desired directions
        self.assertTrue(self.detector.is_pair_in_desired_direction(45, 225, [(135, 315), (45, 225)], 15))

    def test_case_6(self):
        # Test no matching direction
        self.assertFalse(self.detector.is_pair_in_desired_direction(90, 270, [(135, 315), (45, 225)], 15))

if __name__ == '__main__':
    unittest.main()  # Fixed the main call
