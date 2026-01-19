import org.junit.Test;
import org.usfirst.frc3620.GitNess;

/* make a test that does nothing so we never have trouble with Gradle test not finding any tests */
public class GitNessTest {
    @Test
    public void gitness01() {
        System.out.println(GitNess.gitDescription());
        System.out.println(GitNess.gitString());
    }
}
