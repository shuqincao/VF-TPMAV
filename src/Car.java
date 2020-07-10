import java.io.IOException;

public class Car {
    public double[] x;
    public double R;
    public double speed;
    public double[] v;
    public double cosa;
    public double cosb;
    public double zhongliang;
    public double a0;
    public double a1;
    public Car(double R) {
        super();
        this.R = R;
        x = new double[2];
        zhongliang=2000;

        speed=30;
        a0=0;
        a1=0;
        v = new double[2];

        cosa=1;
        cosb=0;
        v[0]=speed*cosa;
        v[1]=speed*cosb;


    }

    public double distance(Car c)
    {
        double a = (this.x[0]-c.x[0])*(this.x[0]-c.x[0])+(this.x[1]-c.x[1])*(this.x[1]-c.x[1]);
        return Math.sqrt(a);
    }


    public double angleCos(Car j){
        double lengthv=Math.sqrt(this.v[0]*this.v[0]+this.v[1]*this.v[1]);
        double lengthx=Math.sqrt(  (j.x[0]-this.x[0])*(j.x[0]-this.x[0])
                +(j.x[1]-this.x[1])*(j.x[1]-this.x[1]) );
        double cos=(  v[0]*(j.x[0]-this.x[0])  +  v[1]*(j.x[1]-this.x[1])  )/(lengthv*lengthx);
        return cos;
    }


    public double[] disv(Car j){
        v = new double[2];
        v[0]=j.v[0]-this.v[0];
        v[1]=j.v[1]-this.v[1];
        return v;
    }
}
