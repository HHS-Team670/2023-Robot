import static org.junit.jupiter.api.Assertions.assertEquals;
import java.beans.Transient;
import java.util.Map;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;


public class VoltageCalculatorTest {
    private VoltageCalculator calculator;

    private Map<String, double[]> testAngles = new HashMap<String, double[]>() {{
        put("vertical", new double[] {90, 180, 180});
        put("horizontal", new double[] {0, 180, 180});
        put("diagonal up right", new double[] {45, 135, 180});
        put("curve downward right", new double[] {90, 135, 135});
        put("diagonal up left", new double[] {225, 135, 180});
        put("horizontal left", new double[] {270, 180, 180});
    }};

    @BeforeEach
    void setUp() {
        
    }

    // public static void main(String[] args) {
    //     VoltageCalculatorTest tester = new VoltageCalculatorTest();
    //     tester.testSimpleJoints();
    // }

    @Test
    @DisplayName("Three simple joints")
    void testSimpleJoints() {
        ArmSegment simpleShoulder = new ArmSegment(10, 1, 0.5, 3);
        ArmSegment simpleElbow = new ArmSegment(10, 1, 0.5, 2);
        ArmSegment simpleWrist = new ArmSegment(10, 1, 0.5, 1);
        calculator = new VoltageCalculator(simpleShoulder, simpleElbow, simpleWrist);

        for(String key : testAngles.keySet()) {
            System.out.println("Testing " + key + "...");
            double[] voltages = calculator.calculateVoltages(testAngles.get(key));
            System.out.println("voltages: " + voltages.toString());
        }
    }
}
