import java.io.IOException;

public class mainTest {
    /**
     * @param args
     * @throws IOException
     * @author sun
     */
    public static void main(String[] args) throws IOException {
        AFASCar run = new AFASCar(6, 100,10);
        run.doAFAS(599 ,0.25,30,10,30,250,7.5,1.5,10,600,2.5,1.5,10);
        System.out.println();


    }

}
