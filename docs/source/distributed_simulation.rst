Distrubuted Simulation
======================

.. image:: 


Client Server
--------------
UE client server overview. Replication and RPC.


Basic idea
------------
ROS2 node only in the client.
URRROS2SimulationstateClient initialize ROS2Component in client only.


NetworkGameMode and NetworkPlayerController
--------------------------------------------
use simstate client. time sync and RPC of robots.


Time synchronization
--------------------

Client Authority and Server Authority
-------------------------------------

Example
---------
Editor only setting
- LargeMap. 
    - GameMode
    - Player setting
    - 
- sim state client has namespace
- script to spawn and send cmd

Todo
--------
- Server authority
- Delay compensation
