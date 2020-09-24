import re


DATA_PATH = "dataset.dat"

data = open(DATA_PATH, "r")

bak = data.readlines() # a list conteining all lines of our data set


data.close