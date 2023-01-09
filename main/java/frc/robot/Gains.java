package frc.robot;

public class Gains {
    public double P;
    public double I;
    public double D;
    public double F;
    public double IZONE;
    public double PEAK_OUTPUT;
	
	/**
	 * Create a new gains profile for PID
	 * @param _P proportion
	 * @param _I integral
	 * @param _D derivative
	 * @param _F feed forward
	 * @param _IZONE izone
	 * @param _PEAK_OUTPUT peak motor output
	 */
	public Gains(double _P, double _I, double _D, double _F, int _IZONE, double _PEAK_OUTPUT){
		P = _P;
		I = _I;
		D = _D;
		F = _F;
		IZONE = _IZONE;
		PEAK_OUTPUT = _PEAK_OUTPUT;
	}
}
