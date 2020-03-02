import sys
import re
import fileinput

if len(sys.argv) != 5 and len(sys.argv) != 3:
    print "Wrong command, sample commands: \n python setproxy.py server port username password \n OR \n python setproxy.py server port"

else:
    proxy = sys.argv[1]
    port = sys.argv[2]
    if len(sys.argv) == 5:
        username = sys.argv[3]
        password = sys.argv[4]
        mark = 0
        for line in fileinput.input("/etc/environment", inplace=1):
            if "proxy" in line:
                mark = 1
                line = re.sub(r'(.*)_proxy=(.*)', r'\1_proxy="\1://'+username+':'+password+'@'+proxy+':'+port+"/\"\n", line.rstrip())
            sys.stdout.write(line)

        if mark == 0:
            file1 = open("/etc/environment", "w")
            file1.write("PATH=\"/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games\"\n")
            file1.write("http_proxy=\"http://"+username+":"+password+"@"+proxy+":"+port+"/\"\n")
            file1.write("https_proxy=\"https://"+username+":"+password+"@"+proxy+":"+port+"/\"\n")
            file1.write("ftp_proxy=\"ftp://"+username+":"+password+"@"+proxy+":"+port+"/\"\n")
            file1.write("socks_proxy=\"socks://"+username+":"+password+"@"+proxy+":"+port+"/\"\n")
            file1.close()

        mark = 0
        for line in fileinput.input("/etc/apt/apt.conf", inplace=1):
            if "Acquire::" in line and "Cache" not in line:
                mark = 1
                line = re.sub(r'Acquire::(.*)::proxy (.*)', r'Acquire::\1::proxy "\1://'+username+":"+password+"@"+proxy+":"+port+"/\";\n", line.rstrip())
            sys.stdout.write(line)

        if mark == 0:
            file1 = open("/etc/apt/apt.conf", "w")
            file1.write("Acquire::http::proxy \"http://"+username+":"+password+"@"+proxy+":"+port+"/\";\n")
            file1.write("Acquire::https::proxy \"https://"+username+":"+password+"@"+proxy+":"+port+"/\";\n")
            file1.write("Acquire::ftp::proxy \"ftp://"+username+":"+password+"@"+proxy+":"+port+"/\";\n")
            file1.write("Acquire::http::No-Cache \"True\";\n")
            file1.write("Acquire::socks::proxy \"socks://"+username+":"+password+"@"+proxy+":"+port+"/\";\n")
            file1.close()

        mark = 0
        for line in fileinput.input("/etc/bash.bashrc", inplace=1):
            if "export" in line:
                mark = 1
                line = re.sub(r'export (.*)_proxy=(.*)', r'export \1_proxy=\1://'+username+':'+password+'@'+proxy+':'+port+'\n', line.rstrip())
            sys.stdout.write(line)

        if mark == 0:
            file1 = open("/etc/bash.bashrc", "a")
            file1.write("\n\nexport http_proxy=http://"+username+":"+password+"@"+proxy+":"+port+"\n")
            file1.write("export https_proxy=https://"+username+":"+password+"@"+proxy+":"+port+"\n")
            file1.write("export ftp_proxy=ftp://"+username+":"+password+"@"+proxy+":"+port+"\n")
            file1.close()


        mark = 0
        for line in fileinput.input("/etc/wgetrc", inplace=1):
            if not line.startswith("#") and "proxy" in line:
                mark = 1
                line = re.sub(r'(.*)_proxy=(.*)//(.*)', r'\1_proxy=\1://'+username+':'+password+'@'+proxy+':'+port+'\n', line.rstrip())
            sys.stdout.write(line)

        if mark == 0:
            file1 = open("/etc/wgetrc", "a")
            file1.write("\n\nhttp_proxy=http://"+username+":"+password+"@"+proxy+":"+port+"\n")
            file1.write("https_proxy=https://"+username+":"+password+"@"+proxy+":"+port+"\n")
            file1.write("ftp_proxy=ftp://"+username+":"+password+"@"+proxy+":"+port+"\n")
            file1.close()

    else:
        mark = 0
        for line in fileinput.input("/etc/environment", inplace=1):
            if "proxy" in line:
                mark = 1
                line = re.sub(r'(.*)_proxy=(.*)', r'\1_proxy="\1://'+proxy+':'+port+"/\"\n", line.rstrip())
            sys.stdout.write(line)

        if mark == 0:
            file1 = open("/etc/environment", "w")
            file1.write("PATH=\"/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games\"\n")
            file1.write("http_proxy=\"http://"+proxy+":"+port+"/\"\n")
            file1.write("https_proxy=\"https://"+proxy+":"+port+"/\"\n")
            file1.write("ftp_proxy=\"ftp://"+proxy+":"+port+"/\"\n")
            file1.write("socks_proxy=\"socks://"+proxy+":"+port+"/\"\n")
            file1.close()

        mark = 0
        for line in fileinput.input("/etc/apt/apt.conf", inplace=1):
            if "Acquire::" in line and "Cache" not in line:
                mark = 1
                line = re.sub(r'Acquire::(.*)::proxy (.*)', r'Acquire::\1::proxy "\1://'+proxy+":"+port+"/\";\n", line.rstrip())
            sys.stdout.write(line)

        if mark == 0:
            file1 = open("/etc/apt/apt.conf", "w")
            file1.write("Acquire::http::proxy \"http://"+proxy+":"+port+"/\";\n")
            file1.write("Acquire::https::proxy \"https://"+proxy+":"+port+"/\";\n")
            file1.write("Acquire::ftp::proxy \"ftp://"+proxy+":"+port+"/\";\n")
            file1.write("Acquire::http::No-Cache \"True\";\n")
            file1.write("Acquire::socks::proxy \"socks://"+proxy+":"+port+"/\";\n")
            file1.close()

        mark = 0
        for line in fileinput.input("/etc/bash.bashrc", inplace=1):
            if "export" in line:
                mark = 1
                line = re.sub(r'export (.*)_proxy=(.*)', r'export \1_proxy=\1://'+proxy+':'+port+'\n', line.rstrip())
            sys.stdout.write(line)

        if mark == 0:
            file1 = open("/etc/bash.bashrc", "a")
            file1.write("\n\nexport http_proxy=http://"+proxy+":"+port+"\n")
            file1.write("export https_proxy=https://"+proxy+":"+port+"\n")
            file1.write("export ftp_proxy=ftp://"+proxy+":"+port+"\n")
            file1.close()


        mark = 0
        for line in fileinput.input("/etc/wgetrc", inplace=1):
            if not line.startswith("#") and "proxy" in line:
                mark = 1
                line = re.sub(r'(.*)_proxy=(.*)//(.*)', r'\1_proxy=\1://'+proxy+':'+port+'\n', line.rstrip())
            sys.stdout.write(line)

        if mark == 0:
            file1 = open("/etc/wgetrc", "a")
            file1.write("\n\nhttp_proxy=http://"+proxy+":"+port+"\n")
            file1.write("https_proxy=https://"+proxy+":"+port+"\n")
            file1.write("ftp_proxy=ftp://"+proxy+":"+port+"\n")
