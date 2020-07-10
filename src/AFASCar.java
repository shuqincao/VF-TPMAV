import java.io.IOException;
import java.math.BigDecimal;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

public class AFASCar {


    private int carNum;
    private int[] choosed;
    private double R;

    public int scopelength;
    Car[] car;


    public AFASCar(int carNum, double R,int kuan)  throws IOException {
        super();
        this.carNum = carNum;
        this.R=R;
        car=new Car[carNum];
        choosed = new int[carNum];
        init(kuan);
    }

    private void init(int kuan) throws IOException {
        List list=new ArrayList();
        for(int i=0;i<carNum;i++)
        {
            car[i] = new Car(R);
            choosed[i]=-1;



           if(i==0){
                car[i].x[0]=0;
                car[i].x[1]=2.5;

            }
            if(i==1){
                car[i].x[0]=50;
                car[i].x[1]=2.5;

            }
            if(i==2){
                car[i].x[0]=100;
                car[i].x[1]=2.5;

            }
            if(i==3){
                car[i].x[0]=0;
                car[i].x[1]=7.5;

            }
            if(i==4){
                car[i].x[0]=50;
                car[i].x[1]=7.5;

            }
            if(i==5){
                car[i].x[0]=100;
                car[i].x[1]=7.5;

            }




            System.out.println("初始化时，第"+(i+1)+"辆车子的横坐标："+car[i].x[0]+",纵坐标："+car[i].x[1]+",速度为"+car[i].speed);

            list.add(car[i].x[1]);



        }
    }


    public void
    doAFAS(int num,double diftime,double fanwei,int kuan,double bingxing,double p0,double p1,double pR,double vp, double x0,double x1,double xR,double vx) throws IOException
    {
        int count=0;
        double[][] matrix=new double[carNum][num+1];
        double[][] matriy = new double[carNum][num+1];
        double[][] matriv = new double[carNum][num+1];
        double[] obstaclex=new double[num+1];
        double[] obstacley=new double[num+1];
        double[] xobstaclex=new double[num+1];
        double[] xobstacley=new double[num+1];
        while(count<=num)
        {
            double[][] Fsum=new double[carNum][2];
            double[] li=new double[carNum];

            if(count==0){
                obstaclex[count]=p0;
                obstacley[count]=p1;
                xobstaclex[count]=x0;
                xobstacley[count]=x1;
            }else{
                obstaclex[count]=obstaclex[count-1]+vp/3.6*diftime;
                p0=obstaclex[count];
                obstacley[count]=obstacley[count-1];
                p1=obstacley[count];

                xobstaclex[count]=xobstaclex[count-1]+vx/3.6*diftime;
                x0=xobstaclex[count];
                xobstacley[count]=xobstacley[count-1];
                x1=xobstacley[count];
            }
            System.out.println("第"+ count+"次时，p障碍物横坐标"+obstaclex[count]+",纵坐标："+obstacley[count]);
            System.out.println("第"+ count+"次时，x障碍物横坐标"+xobstaclex[count]+",纵坐标："+xobstacley[count]);

            for(int i=0;i<carNum;i++){
                if(count==0){
                    matrix[i][count]=car[i].x[0];
                    matriy[i][count]=car[i].x[1];
                    matriv[i][count]=car[i].speed;
                }else{
                    car[i].v[0]=3.6*(car[i].v[0]/3.6+car[i].a0*(diftime));
                    car[i].v[1]=3.6*(car[i].v[1]/3.6+car[i].a1*(diftime));
                    if(Fsum[i][1]==0){
                        car[i].v[1]=0;
                    }
                    car[i].speed=Math.sqrt(car[i].v[0]*car[i].v[0]+car[i].v[1]*car[i].v[1]);
                    car[i].cosa=car[i].v[0]/car[i].speed;
                    car[i].cosb=car[i].v[1]/car[i].speed;
                    car[i].x[0]=car[i].x[0]+car[i].v[0]*(diftime)/3.6+0.5*car[i].a0*(diftime)*(diftime);
                    car[i].x[1]=car[i].x[1]+car[i].v[1]*(diftime)/3.6+0.5*car[i].a1*(diftime)*(diftime);
                    matrix[i][count]=car[i].x[0];
                    matriy[i][count]=car[i].x[1];
                    matriv[i][count]=car[i].speed;
                    System.out.println("第"+ count+"次时，第"+(i+1)+"辆横坐标："+car[i].x[0]+",纵坐标："+car[i].x[1]);
                    System.out.println("第"+ count+"次时，第"+(i+1)+"速度"+car[i].speed);
                }
            }

            for(int i=0;i<carNum;i++)
            {

                double F1[]=align(i,count,diftime);

                double F2[]=repulsive(i,count,fanwei,kuan);
                double F6[]=attractive(i,bingxing,fanwei,kuan);

                double F3[]=speedControl(i,count);

                double F4[]= road(i,count,kuan);

                double F8[]=laneCenter(i,kuan);

                double F5[]= propulsive(i);


                List list1=obstacle(count,i,p0,p1,pR,fanwei,kuan,vp);
                if (Double.parseDouble(String.valueOf(list1.get(0)))!=-1){
                    F8[1]=0;
                }
                double[] F7=new double[2];
                F7[0]=Double.parseDouble(String.valueOf(list1.get(1)));
                F7[1]=Double.parseDouble(String.valueOf(list1.get(2)));

                List list2=obstacle(count,i,x0,x1,xR,fanwei,kuan,vx);
                if (Double.parseDouble(String.valueOf(list2.get(0)))!=-1){
                    F8[1]=0;
                }
                double[] F9=new double[2];
                F9[0]=Double.parseDouble(String.valueOf(list2.get(1)));
                F9[1]=Double.parseDouble(String.valueOf(list2.get(2)));
                Fsum[i][0]=(F1[0]+F2[0]+F3[0]+F4[0]+F5[0]+F6[0]+F7[0]+F8[0]+F9[0])/7;
                Fsum[i][1]=(F1[1]+F2[1]+F3[1]+F4[1]+F5[1]+F6[1]+F7[1]+F8[1]+F9[1])/7;
                if(F2[0]==0&&F2[1]==0&&F4[0]==0&&F4[1]==0&&F6[0]==0&&F6[1]==0&&F7[1]==0&&F8[1]==0&&F9[1]==0){

                    if(Math.abs(car[i].speed-getaverageSpeed(i))<=0.5&&car[i].speed<=30+0.5&&car[i].speed>15-0.5&&Math.abs(car[i].v[0]/car[i].speed-getaveragedirection(i))<=0.001){
                        Fsum[i][1]=0;
                    }
                }
                li[i]=Math.round(Math.sqrt(Fsum[i][0]*Fsum[i][0]+Fsum[i][1]*Fsum[i][1]));
                if(li[i]!=0){
                    car[i].a0=Fsum[i][0]/car[i].zhongliang;
                    car[i].a1=Fsum[i][1]/car[i].zhongliang;
                }else{
                    car[i].a0=0;
                    car[i].a1=0;
                }
            }
            count++;
        }
        for(int i=0;i<carNum;i++){
            System.out.println((i+1)+"车辆横坐标");
            for(int j=0;j<num+1;j++){
                System.out.print(matrix[i][j]+",");
            }
            System.out.println();
        }
        System.out.println();
        for(int i=0;i<carNum;i++){
            System.out.println((i+1)+"车辆纵坐标");
            for(int j=0;j<num+1;j++){
                System.out.print(matriy[i][j]+",");
            }
            System.out.println();
        }
        System.out.println();
        System.out.println("p障碍物横坐标");
        for(int i=0;i<num+1;i++){
            System.out.print(obstaclex[i]+",");
        }
        System.out.println();
        System.out.println("p障碍物纵坐标");
        for(int i=0;i<num+1;i++){
            System.out.print(obstacley[i]+",");
        }
        System.out.println();
        System.out.println("x障碍物横坐标");
        for(int i=0;i<num+1;i++){
            System.out.print(xobstaclex[i]+",");
        }
        System.out.println();
        System.out.println("x障碍物纵坐标");
        for(int i=0;i<num+1;i++){
            System.out.print(xobstacley[i]+",");
        }
        System.out.println();
        for(int i=0;i<carNum;i++){
            System.out.println((i+1)+"车辆速度");
            for(int j=0;j<num+1;j++){
                System.out.print(matriv[i][j]+",");
            }
            System.out.println();
        }
        System.out.println();
        double[][][] matrijj = new double[carNum][carNum][num+2];
        for(int i=0;i<carNum;i++) {
            for (int j = i+1; j < carNum; j++) {
                System.out.println((i+1) + "车与" + (j+1) + "车的车间距变化");
                for(int k=0;k<num+1;k++){

                    matrijj[i][j][k]=Math.sqrt((matrix[i][k]-matrix[j][k])*(matrix[i][k]-matrix[j][k])+(matriy[i][k]-matriy[j][k])*(matriy[i][k]-matriy[j][k]));
                    System.out.print(matrijj[i][j][k]+",");
                }
                System.out.println();
            }
        }

        System.out.println();
        double[][] matriip = new double[carNum][num+2];
        for(int i=0;i<carNum;i++) {
            System.out.println((i+1) + "车与P间距变化");
                for(int k=0;k<num+1;k++){

                    matriip[i][k]=Math.sqrt((matrix[i][k]-obstaclex[k])*(matrix[i][k]-obstaclex[k])+(matriy[i][k]-obstacley[k])*(matriy[i][k]-obstacley[k]));
                    System.out.print(matriip[i][k]+",");
                }
                System.out.println();

        }

        System.out.println();
        double[][] matriix = new double[carNum][num+2];
        for(int i=0;i<carNum;i++) {
            System.out.println((i+1) + "车与O间距变化");
            for(int k=0;k<num+1;k++){

                matriix[i][k]=Math.sqrt((matrix[i][k]-xobstaclex[k])*(matrix[i][k]-xobstaclex[k])+(matriy[i][k]-xobstacley[k])*(matriy[i][k]-xobstacley[k]));
                System.out.print(matriix[i][k]+",");
            }
            System.out.println();

        }
    }
    private double[] propulsive(int i) throws IOException {
        double[] F=new double[2];
        F[0]=car[i].zhongliang*car[i].v[0]/car[i].speed;
        F[1]=car[i].zhongliang*car[i].v[1]/car[i].speed;
        return F;
    }
    private double[] align(int i,int count,double diftime) throws IOException {
        double[] F=new double[2];
        F[1]=0;F[0]=0;
        double[] v_=new double[2];
        v_[0]=0;v_[1]=0;
        int linjusum=getScopecar(i);
        if(linjusum!=0){
            for(int k=0;k<carNum;k++){
                double dik=car[i].distance(car[k]);
                if(i!=k&&dik<=car[i].R){
                    double cos=car[i].angleCos(car[k]);
                    double f=1-((1-cos)/2)*((1-cos)/2);
                    double c=1-((1 - Math.cos(Math.PI *dik/car[i].R))/2)* ((1 - Math.cos(Math.PI *dik/car[i].R))/2 )  ;
                    v_[0]+=c*f*(car[k].v[0]-car[i].v[0]);
                    v_[1]+=c*f*(car[k].v[1]-car[i].v[1]);
                }
            }
            v_[0]=v_[0]/linjusum;
            v_[1]=v_[1]/linjusum;
            F[0]=car[i].zhongliang*v_[0];
            F[1]=car[i].zhongliang*v_[1];

        }
        return F;
    }

    private  double[] attractive(int i,double bingxing,double fanwei,double kuan){
        double[] F=new double[2];
        F[1]=0;F[0]=0;
        double getidui[]=getzuijin(i,kuan);


        int k=(int)getidui[0];
        if(k!=-1 && getidui[1]>bingxing+fanwei){
            double dik=getidui[1];
            double[] direcion=new double[2];
            direcion[0]=0;
            direcion[1]=0;
            direcion[0]=(car[k].x[0]-car[i].x[0])/dik;
            direcion[1]=(car[k].x[1]-car[i].x[1])/dik;
            double b=Math.exp((dik)*(dik)/(2*car[i].R*car[i].R)-1);
            double cos=car[i].angleCos(car[k]);
            double f=1-((1-cos)/2)*((1-cos)/2);
            F[0]+=2*(b+1)*(f+1)*direcion[0];
            F[1]+=2*(b+1)*(f+1)*direcion[1];
        }
        F[0]=F[0]*car[i].zhongliang;
        F[1]=F[1]*car[i].zhongliang;
        return F;

    }
    private  double[] repulsive(int i,int count,double fanwei,double kuan) throws IOException {
        double[] F=new double[2];
        F[1]=0;F[0]=0;
        double[] direcion=new double[2];
        direcion[0]=0;
        direcion[1]=0;
        for(int k=0;k<carNum ;k++){
            if(i!=k && getLane(i,kuan)==getLane(k,kuan)){
                double dik=car[i].distance(car[k]);
                if(dik<=car[i].R){
                    double cos=car[i].angleCos(car[k]);
                    double f=1-((1-cos)/2)*((1-cos)/2);
                    direcion[0]=(car[k].x[0]-car[i].x[0])/dik;
                    direcion[1]=(car[k].x[1]-car[i].x[1])/dik;
                    if(dik<fanwei){
                        double b=Math.exp(-dik*dik/(fanwei*fanwei)*2);
                        F[0]-=2*(b+1)*(f+1)*direcion[0];
                        F[1]-=2*(b+1)*(f+1)*direcion[1];
                    }
                }
            }
            if(i!=k && getLane(i,kuan)!=getLane(k,kuan)){
                double dik=car[i].distance(car[k]);
                if(dik<=car[i].R){
                    double cos=car[i].angleCos(car[k]);
                    double f=1-((1-cos)/2)*((1-cos)/2);
                    direcion[0]=(car[k].x[0]-car[i].x[0])/dik;
                    direcion[1]=(car[k].x[1]-car[i].x[1])/dik;
                    if(dik<3.5){
                        double b=Math.exp(-dik*dik/(2*3.5*3.5));
                        F[0]-=2*(b+1)*(f+1)*direcion[0];
                        F[1]-=2*(b+1)*(f+1)*direcion[1];
                    }
                }
            }
        }
        F[0]=F[0]*car[i].zhongliang;
        F[1]=F[1]*car[i].zhongliang;
        return F;
    }


    public double[] speedControl(int i,int count){
        double[] F=new double[2];
        double v=car[i].speed;
        if(v<15){
            F[0]=car[i].zhongliang*(15-v+1)*car[i].cosa;
            F[1]=car[i].zhongliang*(15-v+1)*car[i].cosb;
        }
        if(v>30){
            F[0]=-car[i].zhongliang*(v-30+1)*car[i].cosa;
            F[1]=-car[i].zhongliang*(v-30+1)*car[i].cosb;
        }
        return F;
    }



    public List  obstacle(int count,int i,double p0,double p1,double pR,double fanwei,double kuan,double vp){
        double[] F=new double[2];
        F[1]=0;F[0]=0;
        List list=new ArrayList();
        list.add(0,-1);
        list.add(1,-1);
        list.add(2,-1);
        double[] direction=new double[2];
        direction[1]=0;direction[0]=0;
        double dip=Math.sqrt((car[i].x[0]-p0)*(car[i].x[0]-p0)+(car[i].x[1]-p1)*(car[i].x[1]-p1));
        double Ysafe = pR + 1 ;
        double Xsafe = pR + 2.5 + car[i].v[0]/3.6 * 0.25 + (((car[i].v[0]-vp)/3.6 * (car[i].v[0]-vp)/3.6)) / 2;

        if(dip<=car[i].R && (p0+pR>=car[i].x[0]+2.5 ||(p0+pR<car[i].x[0]+2.5&&p0+pR>car[i].x[0]-2.5)) && p1<=kuan/2){
            if(Math.abs(car[i].x[1]-p1)<Ysafe){
                if(getLane(i,kuan)==1&&!hasCar(i,kuan)){
                    F[1] = 35*Math.exp(-((car[i].x[1] - p1) * (car[i].x[1] - p1)) / (2* Ysafe * Ysafe) );
                }
                if(getLane(i,kuan)==1 &&hasCar(i,kuan)){
                    F[0] =- 35*Math.exp(-((car[i].x[0] - p0) * (car[i].x[0] - p0)) / ( 2*Xsafe * Xsafe) );
                }
            }else if(Math.abs(car[i].x[1]-p1)>=Ysafe &&car[i].x[1]<=kuan/2){

                list.set(0,0);
            }if(F[0]!=0){
                list.set(0,0);
            }
        }
        else if(dip<=car[i].R  && (p0+pR>=car[i].x[0]+2.5 ||(p0+pR<car[i].x[0]+2.5&&p0+pR>car[i].x[0]-2.5))&& p1>kuan/2){
            if(count>=152 &&i==1){
                System.out.println();
            }


            if(Math.abs(car[i].x[1]-p1)<Ysafe){
                if(getLane(i,kuan)==2 &&!hasCar(i,kuan)){
                    F[1] = -35*Math.exp(-((car[i].x[1] - p1) * (car[i].x[1] - p1)) / ( 2*Ysafe * Ysafe) );
                }
                if(getLane(i,kuan)==2 &&hasCar(i,kuan)){
                    F[0] =- 35*Math.exp(-((car[i].x[0] - p0) * (car[i].x[0] - p0)) / ( 2*Xsafe * Xsafe) );
                }
            }else if(Math.abs(car[i].x[1]-p1)>=Ysafe &&car[i].x[1]>kuan/2){

                list.set(0,0);
            }
            if(F[0]!=0){
                list.set(0,0);
            }

        }

        F[0]=F[0]*car[i].zhongliang;
        F[1]=F[1]*car[i].zhongliang;
        list.set(1,F[0]);
        list.set(2,F[1]);
        return list;
    }


    private double[] laneCenter(int i,double kuan) throws IOException {
        double[] F=new double[2];

        if(car[i].x[1]>kuan/2-2.25&&car[i].x[1]<=kuan/2){
                double li=8*Math.exp(-((kuan/2-car[i].x[1])/(kuan/2)));
                F[1]=-li*car[i].zhongliang;

        }
        if(car[i].x[1]>kuan/2&&car[i].x[1]<=kuan/2+2.25){
                double li=8*Math.exp(-((car[i].x[1]-kuan/2)/(kuan/2)));
                F[1]=li*car[i].zhongliang;
        }
        F[0]=0;
        return F;
    }

    private double[] road(int i,int count,int kuan) throws IOException {

        double dup=kuan-car[i].x[1];
        double[] F=new double[2];
        if(car[i].x[1]>kuan-2.25){
            double li=12/(dup+1);
            F[1]=-li*car[i].zhongliang;
        }

        if(car[i].x[1]<2.25){
            double li=12/(1+car[i].x[1]);
            F[1]=li*car[i].zhongliang;
        }
        F[0]=0;
        return F;
    }



    private double[] getzuijin(int i,double kuan){
        double[] getidui=new double[2];
        double dik=car[i].R+1;
        int index=-1;
        for(int k=0;k<carNum;k++){
            if(k!=i&&getLane(i,kuan)==getLane(k,kuan)){
                if(dik>car[i].distance(car[k])&&car[i].distance(car[k])<=car[i].R){
                    dik=car[i].distance(car[k]);
                    index=k;
                }
            }
        }
        if(index==-1){
            for(int k=0;k<carNum;k++){
                if(k!=i&&getLane(i,kuan)!=getLane(k,kuan)){
                    if(dik>car[i].distance(car[k])&&car[i].distance(car[k])<=car[i].R){
                        dik=car[i].distance(car[k]);
                        index=k;
                    }
                }
            }
        }

        getidui[0]=index;
        getidui[1]=dik;
        return getidui;
    }

    private double getaverageSpeed(int i){
        double as=0;int num=0;
        for(int j=0;j<carNum;j++)
        {
            if(car[i].distance(car[j])<=car[i].R&& i!=j){
                as+=car[j].speed;
                num++;
            }
        }
        return as/num;
    }
    private double getaveragedirection(int i){
        double ad=0;int num=0;
        for(int j=0;j<carNum;j++)
        {
            if(car[i].distance(car[j])<=car[i].R&& i!=j){
                ad+=car[j].v[0]/car[j].speed;
                num++;
            }
        }
        return ad/num;
    }
    public int getLane(int i,double kuan){
        if(car[i].x[1]>0&&car[i].x[1]<=kuan/2){
            return 1;
        }
        if(car[i].x[1]>kuan/2&&car[i].x[1]<=kuan){
            return 2;
        }
        return -1;
    }

    private int getScopecar(int i) {
        int num = 0;
        for(int j=0;j<carNum;j++)
        {


            if(car[i].distance(car[j])<=car[i].R&& i!=j)
            {
                choosed[j] = j;
                num++;
            }
        }
        return num;
    }



    private Boolean hasCar(int i,double kuan){

        for(int j=0;j<carNum;j++){
            double S=5;
            if(car[j].x[0]>=car[i].x[0]&&car[i].v[0]-car[j].v[0]>0){
                S=5+car[i].v[0]/3.6*0.25+(car[i].v[0]/3.6-car[j].v[0]/3.6)*(car[i].v[0]/3.6-car[j].v[0]/3.6)/2;
            }
            if(car[j].x[0]<car[i].x[0]&&car[j].v[0]-car[i].v[0]>0){
                S=5+car[i].v[0]/3.6*0.25+(car[i].v[0]/3.6-car[j].v[0]/3.6)*(car[i].v[0]/3.6-car[j].v[0]/3.6)/2;
            }

            if(i!=j && Math.abs(car[j].x[0]-car[i].x[0])<S &&getLane(i,kuan)!=getLane(j,kuan)) {
                return true;
            }

        }
        return false;

    }



}
