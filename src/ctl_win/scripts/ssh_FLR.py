import paramiko
import time
class SSHFLR:
    def __init__(self,hostname="FLR",username="uc",password="882466aa"):
        self.hostname = hostname
        self.username = username
        self.password = password
    
    def connect(self):
        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect(hostname=self.hostname, port=22, username=self.username, password=self.password)

    def start(self):
        cmd = 'cd fallen_leaves_robot;git pull'
        stdin, stdout, stderr = self.ssh.exec_command(cmd)
        print(stdout.read())
        time.sleep(2)
        cmd = 'source ~/.bashrc;roscore;roslaunch main main.launch'
        stdin, stdout, stderr = self.ssh.exec_command(cmd)
        print(stdout.read(),stderr.read())
        print("FLR start")
    
    def close(self):
        self.ssh.close()

if __name__ == "__main__":
    test = SSHFLR()
    test.connect()
    test.start()
    time.sleep(60*5)