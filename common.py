# Coloful print.
def print_error(*message):
    print('\033[91m', 'ERROR ', *message, '\033[0m')

def print_ok(*message):
    print('\033[92m', *message, '\033[0m')

def print_info(*message):
    print('\033[93m', *message, '\033[0m')

def str2float(string):
    try:
        f = float(string)
    except:
        a, b = string.strip().split('/')
        f = float(int(a) / int(b))
    return f