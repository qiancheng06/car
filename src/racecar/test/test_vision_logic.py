import unittest

import numpy as np

from racecar.vision_logic import BinaryDebouncer, ClassVote, classify_cone_color


class TestVisionLogic(unittest.TestCase):
    def test_red_requires_multiple_hits_and_clears(self):
        filt = BinaryDebouncer(window=5, min_hits=3, clear_frames=2)
        self.assertFalse(filt.update(True))
        self.assertFalse(filt.update(False))
        self.assertFalse(filt.update(True))
        self.assertTrue(filt.update(True))
        self.assertTrue(filt.update(False))
        self.assertFalse(filt.update(False))

    def test_parking_majority_vote(self):
        vote = ClassVote(window=5, min_votes=3)
        self.assertIsNone(vote.update("A"))
        self.assertIsNone(vote.update("B"))
        self.assertIsNone(vote.update("A"))
        self.assertEqual(vote.update("A"), "A")

    def test_color_classifier_has_unknown_state(self):
        dark = np.zeros((30, 30, 3), dtype=np.uint8)
        red = np.zeros((30, 30, 3), dtype=np.uint8)
        red[:, :] = (0, 0, 255)
        blue = np.zeros((30, 30, 3), dtype=np.uint8)
        blue[:, :] = (255, 0, 0)
        self.assertEqual(classify_cone_color(dark), "unknown")
        self.assertEqual(classify_cone_color(red), "red")
        self.assertEqual(classify_cone_color(blue), "blue")


if __name__ == "__main__":
    unittest.main()
