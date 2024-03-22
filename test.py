import copy 




a = [[1], [2], [3]]
b = copy.copy(a)

b[1] = [5]

print(a)
print(b)