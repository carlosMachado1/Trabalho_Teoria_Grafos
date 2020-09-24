import re


DATA_PATH = "dataset.dat"

data = open(DATA_PATH, "r")

bak = data.readlines()
print(bak)

data.close