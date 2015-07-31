#! /usr/bin/env ruby
# -*- coding: utf-8 -*-
# If you want to start the Microsoft Life Cam or the Gumstix camera e-CAM32
# you should use the corresponding ruby run-script. 
if ARGV.empty?
    puts "ERROR: missing argument: You have to inform the log path"
    exit
end

require 'orocos'
require 'vizkit'
include Orocos
Orocos.initialize

Orocos.run 'filter_apriltag_detector::Task' => 'filter'  do
   
    address = ARGV[0].to_s    
 
    log = Orocos::Log::Replay.open(address)

    filter = TaskContext.get_provides 'filter_apriltag_detector::Task'
    filter.apply_conf_file('./ConfigFiles/filter.yml')
    
    filter.configure

    log.apriltag_detector.marker_poses.connect_to filter.pose_sample
    widget = Vizkit.default_loader.Plot2d
    widget2 = Vizkit.default_loader.Plot2d

    filter.out_sec_diff.connect_to do |samples, _| 
        widget.update(samples[5], "yaw_filter")
        widget.update(filter.threshold[5], "thresh")
    end
    log.apriltag_detector.marker_poses.connect_to do |samples2, _|
        widget2.update(samples2.orientation.yaw, "yaw")
    end
    filter.output.connect_to do |samples3, _| 
        widget2.update(samples3.orientation.yaw, "yaw_filtered")
    end


    filter.start
     
    widget.show
    widget2.show
    Vizkit.control log
    Vizkit.exec
end
