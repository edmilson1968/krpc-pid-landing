import org.javatuples.Triplet;

import krpc.client.Connection;
import krpc.client.services.SpaceCenter;
import krpc.client.services.SpaceCenter.ReferenceFrame;
import krpc.client.services.SpaceCenter.Vessel;

public class PID {

	Double kp, ki, kd, p, i, d, target, clampI, lastMeasure;
	double lastTime;
	
	public PID(Double p, Double i, Double d) {
		this.kp = (p == null ? 1.0 : p);
		this.ki = (i == null ? 0.1 : i);
		this.kd = (d == null ? 0.01 : d);
		this.p = 0.0; this.i = 0.0; this.d = 0.0;
		this.target = 0.0;
		this.clampI = 1.0;
		this.lastTime = System.currentTimeMillis()/1000.0;
		this.lastMeasure = 0.0;
	}
	
	public Double getClampI() {
		return clampI;
	}
	public void setClampI(Double clampI) {
		this.clampI = clampI;
	}
	
	public Double getTarget() {
		return target;
	}
	public void setTarget(Double target) {
		this.target = target;
		this.i = 0.0;
	}

	public Double update(double measure) {
		double now = System.currentTimeMillis()/1000.0;
		double change_in_time = now - lastTime;
		if (change_in_time == 0)
			change_in_time = 1;
		
		double error = target - measure;
		this.p = error;
		this.i += error;
		this.i = clamp_i(this.i);
		this.d = (measure - lastMeasure)/change_in_time;
		this.lastMeasure = measure;
		this.lastTime = now;
		
		return (this.kp * this.p) + (this.ki * this.i) - (this.kd * this.d);
	}
	
	private double clamp_i(double i) {
		if (i > this.clampI)
			return this.clampI;
		else if (i < -this.clampI)
			return -this.clampI;
		else 
			return i;
	}
	
	public static void main(String[] args) throws Exception {
		double targetVelocity = 2.0;   // The value we're trying to limit ourselves to
		Connection connection = Connection.newInstance("TargetVelocity");
		SpaceCenter spaceCenter = SpaceCenter.newInstance(connection);
	    Vessel vessel = spaceCenter.getActiveVessel();

//        ReferenceFrame referenceFrame = vessel.getOrbitalReferenceFrame(); 
		ReferenceFrame refVelocity = ReferenceFrame.createHybrid(connection, 
				vessel.getOrbit().getBody().getReferenceFrame(), vessel.getSurfaceReferenceFrame(),
				vessel.getOrbit().getBody().getReferenceFrame(), vessel.getOrbit().getBody().getReferenceFrame());

		//Flight flightTelemetry = vessel.flight(referenceFrame);
		//Stream<Double> velocidade = connection.addStream(flightTelemetry, "getVerticalSpeed");
		Triplet<Double, Double, Double> velocidade = vessel.flight(refVelocity).getVelocity();		
		
		PID p = new PID(0.25, 0.025, 0.0025);
		p.setClampI(20.0);
		p.setTarget(targetVelocity);

		vessel.getControl().setSAS(true);
		vessel.getControl().setThrottle(1.0f);
		
//		while (vessel.getThrust() > 0.0)
			vessel.getControl().activateNextStage();
		
		while (true) {
			velocidade = vessel.flight(refVelocity).getVelocity();
			Double the_pids_output = p.update(velocidade.getValue0());
			vessel.getControl().setThrottle(the_pids_output.floatValue());
			
			System.out.println(String.format("%f;%f;%f", velocidade.getValue0(), the_pids_output.floatValue(), vessel.getControl().getThrottle()));
			
			Thread.sleep(10);
		}
		
	}

}
