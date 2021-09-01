
import subprocess
import time
import platform

# Create your dictionary class
class my_dictionary(dict):
    # __init__ function
    def __init__(self):
        self = dict()
    # Function to add key:value
    def add(self, key, value):
        self[key] = value

def pingOk(sHost):
    try:
        output = subprocess.check_output(
            "ping -{} 1 {}".format('n' if platform.system().lower() == "windows" else 'c', sHost), shell=True)
    except Exception as e:
        return False

    return True

result = my_dictionary()
with open("/home/ansible/robot") as fp:
    lines = fp.readlines()
    for line in lines:
        result.add(line, pingOk(line))
print (result)
