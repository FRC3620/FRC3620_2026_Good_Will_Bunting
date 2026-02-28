import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

import org.junit.Test;

// make a test that does nothing so we just specify this in build.gradle
// (if we specify nothing, we get everything).
public class BooleanSupplierDemo {
    @Test
    public void doNothing() {
        System.out.println("hi bob");
        Set<BooleanSupplier> all_booleanSuppliers = new HashSet<>();
        all_booleanSuppliers.add(makeSupplierA());
        all_booleanSuppliers.add(makeSupplierB());
        System.out.println(all_booleanSuppliers);

        for (var device : all_booleanSuppliers) {
            boolean value = device.getAsBoolean();
            System.out.println(device + " gives us " + value);
        }


    }

    public void test01(BooleanSupplier bs) {
        boolean value = bs.getAsBoolean();
        System.out.println(bs + " gives us " + value);
    }

    public BooleanSupplier makeSupplierA() {
        return () -> false;

    }

    public BooleanSupplier makeSupplierB() {
        return () -> true;

    }
}
