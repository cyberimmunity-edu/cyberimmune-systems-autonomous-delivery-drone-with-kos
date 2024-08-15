from utils.utils import *

def test_haversine_same_coordinates():
   res = haversine(50, 50, 50, 50)
   assert res == 0