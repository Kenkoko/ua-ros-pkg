/*
 * WubbleTeach.java
 */

package edu.arizona.teach;

import org.jdesktop.application.Application;
import org.jdesktop.application.SingleFrameApplication;
import ros.NodeHandle;
import ros.Ros;


/**
 * The main class of the application.
 */
public class WubbleTeach extends SingleFrameApplication {

    /**
     * At startup create and show the main frame of the application.
     */
    @Override protected void startup() {
        show(new MainWindow(this));
    }

    /**
     * This method is to initialize the specified window by injecting resources.
     * Windows shown in our application come fully initialized from the GUI
     * builder, so this additional configuration is not needed.
     */
    @Override protected void configureWindow(java.awt.Window root) {
    }

    /**
     * A convenient static getter for the application instance.
     * @return the instance of WubbleTeach
     */
    public static WubbleTeach getApplication() {
        return Application.getInstance(WubbleTeach.class);
    }

    static NodeHandle nh;
    /**
     * Main method launching the application.
     */
    public static void main(String[] args) {

        System.out.println("Hello World");

        // THIS
         Ros ros = Ros.getInstance();
         ros.init("teaching_interface");
         nh = ros.createNodeHandle();
/*
        try {
            Ros ros = Ros.getInstance();
            ros.init("teaching_interface");
            nh = ros.createNodeHandle();

//            Publisher<Verbescription> pub = nh.advertise("/pub", new VerbDescription(), 1, true);
//            VerbDescription m = new VerbDescription();
//            m.verb = "go";
//            m.arguments = new String[] {"thing", "place"};
//            pub.publish(m);

            ServiceClient<DescribeMDP.Request, DescribeMDP.Response, DescribeMDP> sc
                    = nh.serviceClient("environment/describe_mdp", new DescribeMDP());
            DescribeMDP.Request req = new DescribeMDP.Request();
        try {
            DescribeMDP.Response resp = sc.call(req);
            for (MDPClassDescription c : resp.description.classes) {
                System.out.println(c.name);
                System.out.println(c.attributes.length);
            }
        } catch (RosException ex) {
            Logger.getLogger(WubbleTeach.class.getName()).log(Level.SEVERE, null, ex);
        }
*/
         // THIS
            launch(WubbleTeach.class, args);

            
            // Maybe a ros.spin() here?
//        } catch (RosException ex) {
//            Logger.getLogger(WubbleTeach.class.getName()).log(Level.SEVERE, null, ex);
//        }
    }
}
