# krpc-pid-landing
java krpc landing script using a PID system

This project uses a PID class to control the landing system like SpaceX Falcon-9.

Generic PID Controller Class Based on the PID recipe at :
    http://code.activestate.com/recipes/577231-discrete-pid-controller/
and the code and discussions in the blog at:
    http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

An instance is created with the format your_pid=PID(P=.0001, I=0.00001, D=0.000001)
Finding the right values for those three gain numbers is called 'tuning' and that's beyond the scope of this doc string!  

Use your_pid.setpoint(X) to set the target output value of the controller. 
    
Regularly call your_pid.update(Y), passing it the input data that the controller should respond to.

output_data = your_pid.update(input_data)


Obviously it's just an incipient part of the job. Falcon-9 routines are much more complex and consider lots of variables and parameters.

Clone the project and copy the .craft file in the "saves/<your-save>/Ships/VAB" folder.


Compiling 
$ javac -d bin -cp lib/krpc-java-0.3.11.jar:lib/protobuf-java-3.4.0.jar:lib/javatuples-1.2.jar:lib/commons-math3-3.6.1.jar -verbose src/TesteSuicideBurnFull.java src/PID.java

Executing
$ java -cp bin:lib/krpc-java-0.3.11.jar:lib/protobuf-java-3.4.0.jar:lib/javatuples-1.2.jar:lib/commons-math3-3.6.1.jar TesteSuicideBurnFull

