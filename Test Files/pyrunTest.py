# Pyrun Test
string = "Test"
data = []
i = 0
while i < 5:
    coord = [-175.0 + 10*i,-180.0,250.0, 180.0,0.0,90.0]
    i = i + 1
    data.append(coord)

print(data[2][:3])