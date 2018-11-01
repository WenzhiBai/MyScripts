import os
filepath = "D:/opencv-3.4.0-build/install/x86/vc15/lib/debug"
files = os.listdir(filepath)

output = open("lib.txt","w+")

for file in files:
    output.write(file)
    output.write('\n')

output.close()
