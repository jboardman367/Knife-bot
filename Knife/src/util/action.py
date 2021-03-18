from util.vec import Vec3, Line, Arc
from typing import Union
from util.object_models import Car
from rlbot.agents.base_agent import SimpleControllerState
import math

def clamp(a,b,c):
    return max(a,min(b,c))

def sign(x) -> int:
    return int(math.copysign(1,x))

def double_log_squish(n:float):
    if n>0:
        return
    elif n<0:
        return
    else:
        return 0

class Action:
    def __init__(self):
        pass

def generate_pd_drive(package, target:Union[Line,Arc], car:Car, target_vel:float=1400) -> SimpleControllerState:
    """
    This is not a proper PD,
    """
    controls = SimpleControllerState()
    line = target
    if hasattr(target, 'center'):
        line = target.tangent(car.location)

    ep = 8
    P = -line.p_vec(car.location)
    ed = 15
    D = abs(car.velocity.scalar_proj(line.v.flat_perp())) * line.v
    ei = 400 / (P.length() + 3)
    I = abs(car.velocity.scalar_proj(line)) * line.v
    controls.steer = clamp(-1, car.orientation.forward.ang_2d(ep*P+ei*I+ed*D), 1)
    #simulate deadzone
    controls.steer = 0 if abs(controls.steer) < 0.01 else controls.steer
    package.renderer.draw_line_3d(car.location, car.location + ep*P+ei*I+ed*D, package.renderer.red())
    package.renderer.draw_line_3d(car.location, car.location + P, package.renderer.blue())
    line.render(package.renderer)

    controls.throttle, controls.boost = throttle_controller(car.velocity.length(),target_vel)

    return controls

def an_actual_pd(package, target:Union[Line,Arc], car:Car, target_vel:float=1400) -> SimpleControllerState:
    controls = SimpleControllerState()
    line = target
    if hasattr(target, 'center'):
        line = target.tangent(car.location)

    front_wheels = car.location + car.orientation.forward*51
    package.renderer.draw_line_3d(Vec3(front_wheels.x,front_wheels.y,0),Vec3(front_wheels.x,front_wheels.y,150),
                                  package.renderer.blue())

    side = sign(line.p_vec(front_wheels).dot(line.v.flat_perp())) #used steer to the correct side
    ep = line.p_vec(front_wheels).length() * side #cross-track error
    #ep = 0 if abs(ep) < 5 else ep #ignore small ep
    P = 0.0050
    ed = car.velocity.scalar_proj(line.v.flat_perp()) #cross-track error rate
    #ed = 0 if abs(ed) < 5 else ed #ignore small ed
    D = 0.0100

    controls.steer = clamp(-1,clamp(-1, ep*P, 1)+clamp(-1, ed*D, 1),1)**3

    #add a force aiming to align steering to 0
    #counter_force = 0.0002 * car.velocity.length()
    #reduced_steer = clamp(-1, controls.steer - sign(controls.steer) * counter_force, 1)
    #controls.steer = reduced_steer if sign(controls.steer)==sign(reduced_steer) else 0

    #Add fake threshold/deadzone
    #controls.steer = 0 if abs(controls.steer<0.001) else controls.steer
    #if int(package.packet.game_info.seconds_elapsed *200) %100 ==0:
        #print(f"{int(abs((ep+1)*(ed+1)))}") #this <1000 is close enough to counting as on line

    controls.throttle, controls.boost = throttle_controller(car.velocity.length(), target_vel)

    return controls

def generate_virx_drive(package, target:Line, car:Car, duration:float, target_vel:float=1400) -> SimpleControllerState:
    """
    Generates a controller that is driving toward a target with a preferred striking line
    Calculation concept credit to Virx
    """
    controls = SimpleControllerState()
    to_r = target.r - car.location

    side_of_line = sign(target.p_vec().dot(to_r))
    offset_vec = side_of_line * to_r.flat_perp().normalized()

    adjustment = to_r.ang_2d(target) * clamp(0.25,duration,2) * 500
    target_vec = to_r + adjustment * offset_vec
    controls.steer = clamp(-1,10*car.orientation.forward.ang_2d(target_vec),1)

    return controls

def throttle_controller(initial:float, target:float):
    """
    This returns a tuple of (Throttle:float, Boost:bool)
    """
    vel_diff = target - initial
    throttle = clamp(-1, (vel_diff ** 2) * sign(vel_diff) / 1000, 1)
    boost = vel_diff >150 and 10 < initial < 2200
    return throttle, boost