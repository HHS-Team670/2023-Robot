import java.util.HashMap;
import java.util.ArrayList;
import java.util.Arrays;

public class VoltageCalculatorTest {
    private VoltageCalculator calculator;

    private HashMap<String, double[]> testAngles;

    public VoltageCalculatorTest() {
        testAngles = new HashMap<String, double[]>();

        testAngles.put("vertical", new double[] {90, 180, 180});
        testAngles.put("horizontal", new double[] {0, 180, 180});
        testAngles.put("diagonal up right", new double[] {45, 180, 180});
        testAngles.put("curve up right", new double[] {90, 135, 315});
        testAngles.put("diagonal up left", new double[] {225, 180, 180});
        testAngles.put("horizontal left", new double[] {180, 180, 180});
    }

    public static void main(String[] args) {
        VoltageCalculatorTest tester = new VoltageCalculatorTest();
        tester.testSimpleJoints();
    }

    void testSimpleJoints() {
        ArmSegment simpleShoulder = new ArmSegment(1, 1, 0.5, 3);
        ArmSegment simpleElbow = new ArmSegment(1, 1, 0.5, 2);
        ArmSegment simpleWrist = new ArmSegment(1, 1, 0.5, 1);
        calculator = new VoltageCalculator(simpleShoulder, simpleElbow, simpleWrist);

        for(String key : testAngles.keySet()) {
            System.out.println("Testing " + key + "...");
            ArrayList<Double> voltages = calculator.calculateVoltages(testAngles.get(key));
            System.out.println("voltages: " + round(voltages).toString());
            System.out.println("");
        }

    }

    public static ArrayList<Double> round(ArrayList<Double> input) {
        ArrayList<Double> roundedDoubles = new ArrayList<Double>();

        for(Double d : input) {
            roundedDoubles.add(((int) (d * 1000.0)) / 1000.0);
        }

        return roundedDoubles;
    }

    public static ArrayList<Double> round(double[] input) {
        ArrayList<Double> arrayList = new ArrayList<Double>();

        for(int i = 0; i < input.length; i++) {
            arrayList.add(input[i]);
        }
        return round(arrayList);
    }
}
