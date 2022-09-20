import os, sys, json

num = 0
for file in os.listdir(sys.argv[1]):
    f = open(os.path.join(sys.argv[1], file), 'r')
    label = json.load(f)
    if len(label['objects']) > 0:
        num+=1

print(num)