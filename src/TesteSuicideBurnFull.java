import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.javatuples.Triplet;

import krpc.client.Connection;
import krpc.client.services.SpaceCenter;
import krpc.client.services.SpaceCenter.Flight;
import krpc.client.services.SpaceCenter.ReferenceFrame;
import krpc.client.services.SpaceCenter.SASMode;
import krpc.client.services.SpaceCenter.SpeedMode;
import krpc.client.services.SpaceCenter.Vessel;
import krpc.client.services.SpaceCenter.VesselSituation;

public class TesteSuicideBurnFull {

	public static void main(String[] args) throws Exception {
		Connection conexao = Connection.newInstance("TesteSuicideBurnFull");
		SpaceCenter centroEspacial = SpaceCenter.newInstance(conexao);
		Vessel nave = centroEspacial.getActiveVessel(); 
		
		ReferenceFrame pontoReferencia = nave.getOrbitalReferenceFrame();
		Flight parametrosVoo = nave.flight(pontoReferencia);

		float heading = 75.0f;
		float apoapsis = 82000.0f;
		
		nave.getAutoPilot().targetPitchAndHeading(90, heading);
		nave.getAutoPilot().engage();
		nave.getControl().setThrottle(1);
		Thread.sleep(10);
		nave.getControl().setRCS(true);
		System.out.print("3");
		Thread.sleep(1000);
		System.out.print("..2");
		Thread.sleep(1000);
		System.out.print("..1");
		Thread.sleep(1000);

		System.out.print("..DECOLAGE!\n");
		nave.getControl().activateNextStage();
		Thread.sleep(3000);
		nave.getAutoPilot().targetPitchAndHeading(90, heading);
		nave.getControl().setGear(false);

		//curva de ajuste do pitch
		double[] x = new double[] {0.0, 5.0, 10.0, 20.0, 40.0, 100.0, 150.0};
		double[] y = new double[] {85.0, 70.0, 55.0, 45.0, 30.0, 10.0, 0.0};
		
		boolean verificaBooster = ! nave.getParts().withName("solidBooster1-1").isEmpty();
		System.out.println("verificaBooster:" + verificaBooster );
		while (nave.getOrbit().getApoapsisAltitude() < apoapsis) {
	        Double pitch = linearInterp(parametrosVoo.getMeanAltitude()/1000.0, x, y);
	        nave.getAutoPilot().targetPitchAndHeading(pitch.floatValue(), heading);
	        Double throttle = (double) (2 * nave.getOrbit().getBody().getSurfaceGravity() * nave.getMass() / nave.getAvailableThrust()); // Limitar a uma aceleracao de 2g
	        nave.getControl().setThrottle( throttle.floatValue() );
	        
	        if (verificaBooster && (nave.getResources().amount("SolidFuel") < 0.1f)) {
	    		System.out.println("Booster separation");
	    		nave.getControl().activateNextStage();
	    		verificaBooster = false;
	        }
	        
	        Thread.sleep(10);
		}
		
		System.out.println("Launch stage separation");
		nave.getControl().setThrottle(0);
		
		Thread.sleep(1000);
		nave.getControl().activateNextStage();
		nave.getAutoPilot().disengage();
		System.out.println("fase sub-orbital finalizada");

		
		System.out.println("fase retro-burn iniciada");
		System.out.println("autopilot.SAS=true");
		nave.getAutoPilot().setSAS(true);
		Thread.sleep(1000);

	    System.out.println("control.SASMode=retrograde");
	    nave.getAutoPilot().setSASMode(SASMode.RETROGRADE);
	    Thread.sleep(10000);
	    nave.getControl().setRCS(false);

		ReferenceFrame refVelocidade = ReferenceFrame.createHybrid(conexao, 
				nave.getOrbit().getBody().getReferenceFrame(), 
				nave.getSurfaceReferenceFrame(),
				nave.getOrbit().getBody().getReferenceFrame(), 
				nave.getOrbit().getBody().getReferenceFrame());
		
	    double speed, altitude;
	    do {
	    	Thread.sleep(10);
	    	speed = nave.flight(refVelocidade).getSpeed();
	    	
	    	altitude = nave.flight(pontoReferencia).getSurfaceAltitude();
	    	//System.out.println(String.format("%f;%f", speed,altitude));
	    	if (altitude < 45000.0)
	    		break;
	    	
	    } while ((speed < 1450));

	    System.out.println("SurfaceSpeed...");
	    nave.getControl().setSpeedMode(SpeedMode.SURFACE);
	    //entry-burn
	    nave.getControl().setThrottle(1);
	    //long init = System.currentTimeMillis();
	    
	    float oxiini = (nave.getResources().amount("Oxidizer"));
	    do {Thread.sleep(10);} while ((nave.getResources().amount("Oxidizer") > (oxiini * 0.7)));
	    System.out.println("time is up!");
	    nave.getControl().setThrottle(0);
	    Thread.sleep(2000);

	    System.out.println("airbrakes on");
	    nave.getControl().setBrakes(true);

	    System.out.println("fase retro-burn finalizada");

		System.out.println("fase suicide-burn e pouso iniciada");
	    
		pontoReferencia = nave.getOrbitalReferenceFrame();
		
		refVelocidade = ReferenceFrame.createHybrid(conexao, 
				nave.getOrbit().getBody().getReferenceFrame(), 
				nave.getSurfaceReferenceFrame(),
				nave.getOrbit().getBody().getReferenceFrame(), 
				nave.getOrbit().getBody().getReferenceFrame());
		
		Triplet<Double, Double, Double> velocidade;
		
		float aceleracaoGravidade = nave.getOrbit().getBody().getSurfaceGravity();
		
	    //System.out.println("altitudeSuicide,altitude,velocidade,velSquared,acelMax");
		while (true) {
			double acelMax = nave.getAvailableThrust() / nave.getMass() - aceleracaoGravidade;
			//acelMax = (naveTWR * aceleracaoGravidade) - aceleracaoGravidade;
			velocidade = nave.flight(refVelocidade).getVelocity();
			double vel = velocidade.getValue0();
			double velSq = Math.pow(vel, 2);
			altitude = nave.flight(pontoReferencia).getSurfaceAltitude();
			double altSuicide = (altitude * aceleracaoGravidade + 0.5 * velSq) / acelMax; //estimativa de tamanho da nave
			
			//System.out.println(String.format("%f;%f;%f;%f;%f",altSuicide,altitude,vel,velSq,acelMax));
			if (altitude < 2000 && altitude <= altSuicide) {
				System.out.println("Burn baby, burn!");
				break;
			}
		}
		nave.getControl().setThrottle(0.6f);
		nave.getControl().setGear(true);
		
		do {
//			double acelMax = naveAtual.getAvailableThrust() / naveAtual.getMass() - aceleracaoGravidade;
			//acelMax = (naveTWR * aceleracaoGravidade) - aceleracaoGravidade;
			velocidade = nave.flight(refVelocidade).getVelocity();
//			double vel = velocidade.getValue0();
//			double velSq = Math.pow(vel, 2);
//			double altitude = altitudeSuperficie.get();
//			double altSuicide = (altitude * aceleracaoGravidade + 0.5 * velSq) / acelMax - 10; //estimativa de tamanho das pernas de pouso
			
			//System.out.println(String.format("%f;%f;%f;%f;%f",altSuicide,altitude,vel,velSq,acelMax));
			Thread.sleep(1);
		} while (velocidade.getValue0() < -2.0);
		System.out.println("Preparando o hoverslam...");
		nave.getControl().setRCS(true);

        Double alt_kP = 0.25;Double alt_kI = 0.25; Double alt_kD = 0.025;
        //initialise PID
        PID pid = new PID(alt_kP, alt_kI, alt_kD);
		//pid.setClampI(1.0);
		//pid.setTarget(-4.0);

        while ( true ) {
        	if (nave.getSituation() == VesselSituation.SPLASHED || nave.getSituation() == VesselSituation.LANDED)
        		break;
        	
    		velocidade = nave.flight(refVelocidade).getVelocity();
    		altitude = nave.flight(pontoReferencia).getSurfaceAltitude();
    		
    		double safe_descent = altitude / -1.5;

    		if (safe_descent < -25.0)
    			safe_descent = -25.0;
    		
    		pid.setTarget(safe_descent);
			Double pidOutput = pid.update(velocidade.getValue0());
			nave.getControl().setThrottle(pidOutput.floatValue());
			
//			System.out.println(String.format("%f;%f;%f;%s", velocidade.getValue0(), pidOutput.floatValue(), naveAtual.getControl().getThrottle(), naveAtual.getSituation().toString()));
			Thread.sleep(5);
			
	    }
        nave.getControl().setThrottle(0.0f);
		//naveAtual.getControl().setRCS(true);
        nave.getControl().setSASMode(SASMode.STABILITY_ASSIST);
		System.out.println("fase suicide-burn e pouso finalizada");
	    
	    conexao.close();
	}
		
	public static double linearInterp(double xi, double[] x, double[] y) {
		// return linear interpolation of (x,y) on xi
		LinearInterpolator li = new LinearInterpolator();
		PolynomialSplineFunction psf = li.interpolate(x, y);
		double yi = psf.value(xi);
		return yi;
	}


}
