import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.junit.Test;

import org.usfirst.frc3620.RobotParametersBase;
import org.usfirst.frc3620.RobotParametersContainer;
import org.usfirst.frc3620.Minifier;

public class RobotParametersTest {

    static String test_parameters_filename = "src/test/java/RobotParametersTest.json";
    static ObjectMapper objectMapper = new ObjectMapper();

    public static class TestRobotParameters extends RobotParametersBase {
        Double offset;
        public TestRobotParameters() {
            super();
        }

        @Override
        public String toString() {
            StringBuilder sb = new StringBuilder(super.toString());
            sb.setLength(sb.length()-1);
            sb.append (", offset=" + offset);
            sb.append ("]");
            return sb.toString();        }
    }

    @Test
    public void l1_test01() throws IOException {
        String json = Files.readString(Path.of(test_parameters_filename));
        json = Minifier.minify(json);

        List<TestRobotParameters> list2 = objectMapper.readValue(json, new TypeReference<List<TestRobotParameters>>() { });
        System.out.println ("list2: " + list2);

        Map<String, TestRobotParameters> map2 = RobotParametersContainer.makeParameterMap(list2);
        System.out.println ("map2: " + map2);

        List<RobotParametersBase> list3 = objectMapper.readValue(json, new TypeReference<List<RobotParametersBase>>() { });
        System.out.println ("list3: " + list3);

        Map<String, RobotParametersBase> map3 = RobotParametersContainer.makeParameterMap(list3);
        System.out.println ("map3: " + map3);
    }

    @Test
    public void l1_testMinifier() {
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        pw.println ("[");
        pw.println ("   // ignore me");
        pw.println ("   \"a\",");
        pw.println ("   1");
        pw.println ("   /* foo */, 2");
        pw.println ("]");

        String s = sw.toString();
        System.out.println (s);
        String s2 = Minifier.minify(s);
        System.out.println (s2);
    }

    public void l2_class() {
        System.out.println (TestRobotParameters.class.isLocalClass());
        System.out.println (TestRobotParameters.class.isMemberClass());
        System.out.println (TestRobotParameters.class.getEnclosingClass());
        System.out.println (TestRobotParameters.class.getModifiers());
    }
    @Test
    public void l2_testBasic() {
        RobotParametersBase rb = RobotParametersContainer.getRobotParameters(RobotParametersBase.class, test_parameters_filename);
        System.out.println (rb);

        TestRobotParameters trb = RobotParametersContainer.getRobotParameters(TestRobotParameters.class, test_parameters_filename);
        System.out.println (trb);
    }
}