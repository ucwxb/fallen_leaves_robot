import paramiko
import time
class SSHYAHBOOM:
    def __init__(self,hostname="Dofbot",username="dofbot",password="yahboom"):
        self.hostname = hostname
        self.username = username
        self.password = password
    
    def connect(self):
        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect(hostname=self.hostname, port=22, username=self.username, password=self.password)

    def start(self):
        cmd = 'cd fallen_leaves_robot;git pull;roslaunch  arm_control arm_control.launch'
        self.ssh.exec_command(cmd)
        print("Dofbot start")
    
    def close(self):
        self.ssh.close()
