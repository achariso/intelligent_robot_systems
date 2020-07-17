
Vagrant.configure("2") do |config|

  config.vm.box = "irs/ubuntu-14.04"
  config.vm.box_check_update = false

  config.ssh.username = "root"
  config.ssh.password = "robot2016"
  config.ssh.port = 2200
  config.ssh.insert_key = false
  config.ssh.keep_alive = true
  config.ssh.keys_only = false

  config.vm.synced_folder ".", "/home/manos/catkin_ws/src/intelligent_robot_systems_2016"

  config.vm.provider "virtualbox" do |vb|
    vb.gui = true
    vb.memory = "2048"
  end

end
