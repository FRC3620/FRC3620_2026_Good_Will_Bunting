import org.junit.Test;
import org.usfirst.frc3620.Utilities;

public class ExtractTest {
    @Test
    public void doNothing() {
        Integer ff = Utilities.extractPrivateField(Integer.class, C.class, null, "f");
        System.out.println(ff);
    }

    public class C {
        @SuppressWarnings("unused")
        static private int f = 314159;
    }
}
