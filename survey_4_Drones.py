from multiprocessing.pool import ThreadPool as Pool

import setup_path 
import airsim

import sys
import time
import argparse

class SurveyNavigator:
    def __init__(self, args, num):
        self.boxsize = args.size
        self.stripewidth = args.stripewidth
        self.altitude = args.altitude
        self.velocity = args.speed
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True, "Drone{}".format(num))

    def start(self, num):
        print("arming the drone...".format(num))
        self.client.armDisarm(True, "Drone{}".format(num))

        landed = self.client.getMultirotorState().landed_state
        if landed == airsim.LandedState.Landed:
            print("taking off...")
            self.client.takeoffAsync(vehicle_name="Drone{}".format(num)).join()

        landed = self.client.getMultirotorState().landed_state
        if landed == airsim.LandedState.Landed:
            print("takeoff failed - check Unreal message log for details")
            return
        
        # starting points
        if num == 1:
            sp_x = 100 +((num-1)*self.boxsize)
        else:
            sp_x = 100 +((num-1)*self.boxsize) + self.stripewidth 
        
        sp_y = -self.boxsize

        # AirSim uses NED coordinates so negative axis is up.
        x = sp_x
        z = -self.altitude
        

        print("climbing to altitude: " + str(self.altitude))
        self.client.moveToPositionAsync(0, 0, z, self.velocity, vehicle_name="Drone{}".format(num)).join()

        print("flying to first corner of survey box")
        self.client.moveToPositionAsync(sp_x, sp_y, z, self.velocity, vehicle_name="Drone{}".format(num)).join()
        
        # let it settle there a bit.
        self.client.hoverAsync(vehicle_name="Drone{}".format(num)).join()
        time.sleep(2)

        # after hovering we need to re-enabled api control for next leg of the trip
        self.client.enableApiControl(True, "Drone{}".format(num))

        # now compute the survey path required to fill the box 
        path = []
        distance = 0
        while x < sp_x + self.boxsize:
            distance += self.boxsize 
            path.append(airsim.Vector3r(x, self.boxsize, z))
            x += self.stripewidth            
            distance += self.stripewidth 
            path.append(airsim.Vector3r(x, self.boxsize, z))
            distance += self.boxsize 
            path.append(airsim.Vector3r(x, -self.boxsize, z)) 
            x += self.stripewidth  
            distance += self.stripewidth 
            path.append(airsim.Vector3r(x, -self.boxsize, z))
        
        print("starting survey, estimated distance is " + str(distance))
        trip_time = distance / self.velocity
        print("estimated survey time is " + str(trip_time))
        try:
            result = self.client.moveOnPathAsync(path, self.velocity, trip_time*2, airsim.DrivetrainType.ForwardOnly, 
                airsim.YawMode(False,0), self.velocity + (self.velocity/2), 1, vehicle_name="Drone{}".format(num)).join()
        except:
            errorType, value, traceback = sys.exc_info()
            print("moveOnPath threw exception: " + str(value))
            pass

        print("flying back home")
        self.client.moveToPositionAsync(0, 0, z, self.velocity, vehicle_name="Drone{}".format(num)).join()
        
        if z < -5:
            print("descending")
            self.client.moveToPositionAsync(0, 0, -5, 2, vehicle_name="Drone{}".format(num)).join()

        print("landing...")
        self.client.landAsync(vehicle_name="Drone{}".format(num)).join()

        print("disarming.")
        self.client.armDisarm(False, "Drone{}".format(num))

if __name__ == "__main__":
    args = sys.argv
    args.pop(0)
    arg_parser = argparse.ArgumentParser("Usage: survey boxsize stripewidth altitude")
    arg_parser.add_argument("--size", type=float, help="size of the box to survey", default=50)
    arg_parser.add_argument("--stripewidth", type=float, help="stripe width of survey (in meters)", default=10)
    arg_parser.add_argument("--altitude", type=float, help="altitude of survey (in positive meters)", default=30)
    arg_parser.add_argument("--speed", type=float, help="speed of survey (in meters/second)", default=10)
    args = arg_parser.parse_args(args)
    
    # UAV Swarm size
    UAVNum = 4
    
    # pool size
    pool_size = UAVNum  
    
    # define worker function before a Pool is instantiated
    def worker(item):
        try:
            xx = "nav{}".format(item)
            xx = SurveyNavigator(args, item)
            xx.start(item)
        except:
            print('error with item')  

    pool = Pool(pool_size)
    
    for ii in range(1, UAVNum+1, 1):
        pool.apply_async(worker, (ii,))

    pool.close()
    pool.join()    
