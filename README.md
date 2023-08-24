# pylon_camera_sync

The `pylon_camera_sync` repository allow you to grab images in [Synchronous Free Run](https://docs.baslerweb.com/synchronous-free-run) mode, from Basler camers, over the PTP protocol. This mode ensure to acquire images at the same time and at the same frame rate.

## The synchronization process

The activation of the mode has to be done with respect to a beginning timestamp which has to be communicated to all the cameras. The communication of the beginning time require a synchronization process.

The sync process is based on a master-slave interaction. The Master receive an acknowledgment existence message from all the slaves present in the network and communicate back the beginning grabbing timestamp. The below UML diagram illustrate the sequence of performed actions with a specific number. These numbers can be found in the code to better explain the process. Furthermore, 4 blocks were defined in order to separate different logics.

![Camera synchronization process](media/cam-sync-process.png)

<!--
## UML Sequence diagram
## created at https://www.websequencediagrams.com

title Camera Sync Process

opt defineSovereignty()
    Slave->Master: 1) Acknoledgment (ACK) of existence
    Master->Master: 1) wait for additional \n Slaves ACK
end

opt cameraInit()
    note over Slave,Master: Camera initialization: \n - instantiate camera \n - apply startup setting \n - enable chunks \n - setup IEEE1588
end

opt syncCameras()
    Slave->Master: 2) Acknoledgment (ACK) of synchronization
    Master->Master: 2) wait ACKs from all the \n registered Slaves
    
    Slave->Slave: 3) wait from the Master \n the Beginning Timestamp
    Master->Slave: 3) Beginning Timestamp

end

opt enableFreeRunMode()
    note over Slave,Master: Enable the Free Run mode: \n - set the beginning grabbing time \n - enable the SyncFreeRunTimer 
end

-->
