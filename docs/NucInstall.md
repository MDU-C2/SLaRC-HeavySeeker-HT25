## Ubuntu

- version ubuntu server 24.04.3 LTS

### Steps
These steps shows the deviation from the standard alternatives in the installation process

- system language: english
- keyboard layout: swedish
- Ubuntu Server (mimimized)

## After OS installation

- login with specified credentials

- sudo apt update
- sudo apr upgrade -y
- sudo apt install git -y
- git clone https://github.com/MDU-C2/SLaRC-HeavySeeker-HT25.git slarc

- run the installation script in the repository under the folder setup
- (press enter if/when prompted)

## Zero Tier
To join network: `zerotier-cli join [networkID]`
You also have to allow this connection from the zerotier admin panel


## Permissions
You need to allow the user to run `sudo ip link set` commands without requireing a password, this is done by editing the sudoers file.


Fix this with the following steps:
- Run command in cli: `sudo visudo`
- Add `slarc ALL=(ALL) NOPASSWD: /sbin/ip link set *` to the end of the sudoers file
- Save and exit
