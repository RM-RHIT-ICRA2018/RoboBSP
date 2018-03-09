class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def fail(message):
    return (bcolors.FAIL + "[ERROR] " + bcolors.ENDC + str(message))

def warning(message):
    return (bcolors.WARNING + "[WARNING] " + bcolors.ENDC + str(message))

def notice(message):
    return (bcolors.OKGREEN + "[NOTICE] " + bcolors.ENDC + str(message))

def info(message):
    return (bcolors.OKBLUE + "[INFO] " + bcolors.ENDC + str(message))

def access (message):
    return (bcolors.HEADER + "[START] " + bcolors.ENDC + str(message))

