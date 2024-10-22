# Vehicle Framework

Matlab framework for manage single vehicle or vehicles swarm in standalone or cooperative scenario.  
Directory and file organization are the following:
![Class diagram](img/UML_class.png)

## util
### Classes
#### StateSpaceSystem Class
Define a time-continuous dynamic system with a state-space model:
>      dx = A*x + B*u
>      y = C*x + D*u
 
Use a constructor to create a new istance of this class:
> \>\> sys = StateSpaceSystem(A,B,C,D,xi)
>>Initialize a state-space model with A,B,C,D matrix and xi initial conditions.

> \>\> sys = StateSpaceSystem(A,B,C,D)
>>Initialize a state-space model with A,B,C,D matrix and 0 initial conditions.

> \>\> sys = StateSpaceSystem(A,B)
>>Initialize a state-space model with A,B,eye(n),0 matrix and 0 initial conditions.
 
The system simulate with memory, each simulation start from the last state values of the previous simulation. For simulate the system with a costant input u for T seconds use the class function
> \>\> sys.sim(u,T)

The simulation results is stored in the vectors:
* sys.t := time vector
* sys.u := inputs vector
* sys.x := states vector
* sys.y := outputs vector

Reset the simulation to t=0 and initial conditions using the class function
> \>\> sys.reset()

#### NonlinearSystem Class
Define a time-continuous dynamic system with a nonlinear model:
>      dx = f(x,u)
>      y = g(x,u)
 
Use a constructor to create a new istance of this class:
> \>\> sys = NonlinearSystem(f,g,nx,nu,ny,xi)
>> Initialize a nonlinear system with state-transition matlab function f, output matlab function g and xi as initial conditions. States, Input, Output dimension: nx, nu, ny.

> \>\> sys = NonlinearSystem(f,g,nx,nu,ny)
>> Initialize a nonlinear system with state-transition matlab function f, output matlab function g and 0 as initial conditions. States, Input, Output dimension: nx, nu, ny.

> \>\> sys = NonlinearSystem(f,nx,nu)
>> Initialize a nonlinear system with state-transition matlab function f, output matlab function 'full_state' and 0 as initial conditions. States, Input dimension: nx, nu (ny=nx).
 
The system simulate with memory, each simulation start from the last state values of the previous simulation. For simulate the system with a costant input u for T seconds use the class function
> \>\> sys.sim(u,T)

The simulation results is stored in the vectors:
* sys.t := time vector
* sys.u := inputs vector
* sys.x := states vector
* sys.y := outputs vector

Reset the simulation to t=0 and initial conditions using the class function
> \>\> sys.reset()

#### ControlledSystem_LQI Class
Define a time-discrete, with sample time Tc, dynamic closed-loop system:
>      z(k+1) = Φ*z(k) + G*r(k)
>      y(k)  = Hy*z(k)
>      c(k)  = Hc*z(k) + L*r(k)

                                .-------------------------.
                                |   .----.                | x
        +        .-----------.  '-->| x  |    .----.   .--'--.   y
    r --->o----->| Integrator|----->| xc |--->|-Fa |-->| SYS |--.-->
          ^ -    '-----------'      '----'    '----'   '-----'  |
          |                                                     |
          '-----------------------------------------------------'
The closed-loop system is performed with an optimal integral control law (LQI) that provides the tracking of the outputs selected by Cy matrix from a StateSpaceSystem object.<br\>
Use a constructor to create a new istance of this class:
> \>\> ctrl_sys = ControlledSystem_LQI(sys,Tc,Fa,Cy,Phi,G,Hc,L)

The system simulate with memory, each simulation start from the last state values of the previous simulation. For simulate the system with a costant reference r for T seconds use the class function
> \>\> ctrl_sys.sim(r,T)

The simulation results is stored in the vectors:
* ctrl_sys.sys.t := time vector
* ctrl_sys.sys.u := inputs vector
* ctrl_sys.sys.x := states vector
* ctrl_sys.sys.y := outputs vector
* ctrl_sys.xc := controller states
* ctrl_sys.r := references

Reset the simulation to t=0 and initial conditions using the class function
> \>\> ctrl_sys.reset() 

### Functions
#### plot_simulation
Plot simulation data from a StateSpaceSystem or a ControlledSystem_LQI. The default ploted vectors are four:
* figure(1) - states
* figure(2) - inputs
* figure(3) - tracking errors (only for ControlledSystem_LQI)
* figure(4) - references (only for ControlledSystem_LQI)

For plot only some signals use a selector vector:
['x','u','e','r'] := ['states','inputs','errors','references'].
> \>\>plot_simulation(sys)
>> Plot all vectors.

> \>\>plot_simulation(sys,['x','r'])
>> Plot only states and references vectors.

## marine_vehicle
### Classes
#### ControlledVehicle Class
Define an abstraction of a controlled vehicle with a command governor. The vehicle is composed by a ControlledSystem_LQI that rappresent the controlled vehicle for position tracking, and from a CommandGovernor for the computation of references that satify the constrains. The *reference governor* can be also a DistribuitedCommandGovernor for multi-agent or vehicle swarm scenario.<br\>
Use a constructor to create a new istance of this class:
> \>\> vehicle = ControlledVehicle(ctrl_sys)

For add a command governor execute
> \>\> vehicle.cg = CommandGovernor(Phi,G,Hc,L,T,b,Psi,k0,delta,false)

For set the initial position in the plane of a vehicle use the class function
> \>\> vehicle.init_position(obj,x,y)
>> Set vehicle and controller initial conditions relative to a steady position [x,y].

### Modelling Scripts
#### vehicle_model
Dynamic model for a omnidirectional surface marine vehicle without orientation. The used model is like a point free to move on a plane.
>     m*ddx + c*dx = Tx
>     m*ddy + c*dy = Ty

That can be rewritten in a state-space form:
>     dx = dx
>     dy = dy
>     ddx = -c/m*dx + 1/m*Tx
>     ddy = -c/m*dy + 1/m*Ty

Where x and y is the coordinates of the vehicle on the plane, dx and dy the velocities along x and y axis, m is the approximed mass of the vehicle, c the water drag coefficient, while Tx and Ty is the forces generated by the vehicle thruster along x and y axis.<br\>
The model is discretized using *forward Euler* approximation with a specificated time sample Tc. From the discrete model matrix Ad,Bd,Cd,Dd is created a optimal integral control law (LQI) using an augmented state model, and the cloosed-loop matrix Φ,G,Hc,L.

### Applications Scripts
#### vehicle_simple_cg
Simulation scenario for a single vehicle with velocities and thrust constraints using a command governor.

#### vehicle_circle_CG
Simulation scenario for a single vehicle with velocities and thrust constraints using a command governor for tracking a circle.

#### vehicle_cooperation
Configure a little vehicle swarm composed by 3 vehicle organized in a following net
>   1

>  / \\

> 2   3

#### swarm_parallel
Simulation scenario for vehicle swarm with parallel reference recalculation approach.

#### swarm_sequential
Simulation scenario for vehicle swarm with sequential reference recalculation approach.

## CG
### Classes
#### CommandGovernor Class
Command governor for computation of the nearest reference g to the desidered reference r that statify the constrains.
#### DistribuitedCommandGovernor Class
Distribuited command governor for computation of the nearest reference g to the desidered reference r that statify local and global (with other system) constraints.

## tbxmanager
