package smartsystem;

/**
 * Created by ves on 7/1/2015.
 */

import org.xml.sax.Attributes;
import org.xml.sax.SAXException;
import org.xml.sax.helpers.DefaultHandler;

public class RobotSAXHandler extends DefaultHandler {

    private RobotProcess rp;
    private boolean inProcess = false;


    public void startDocument() throws SAXException {
    }

    public void endDocument() throws SAXException {
    }

    public void startElement(String namespaceURI, String localName,
                             String qName, Attributes attrs)
            throws SAXException {

        double tempDouble = 0;

        if (qName.equals("Process")) {
            inProcess = true;
            rp = new RobotProcess();

        } else if (qName.equals("RobotPose")) {

            //trimming decimal places
            tempDouble = Double.parseDouble(attrs.getValue(0));
            tempDouble = (long) (tempDouble * 1e2) / 1e2;
            rp.setToolX(Double.toString(tempDouble));

            tempDouble = Double.parseDouble(attrs.getValue(1));
            tempDouble = (long) (tempDouble * 1e2) / 1e2;
            rp.setToolY(Double.toString(tempDouble));

            tempDouble = Double.parseDouble(attrs.getValue(2));
            tempDouble = (long) (tempDouble * 1e2) / 1e2;
            rp.setToolZ(Double.toString(tempDouble));

        } else if (qName.equals("RobotForces")) {

            tempDouble = Double.parseDouble(attrs.getValue(0));
            tempDouble = (long) (tempDouble * 1e2) / 1e2;
            rp.setForceX(Double.toString(tempDouble));

            tempDouble = Double.parseDouble(attrs.getValue(1));
            tempDouble = (long) (tempDouble * 1e2) / 1e2;
            rp.setForceY(Double.toString(tempDouble));

            tempDouble = Double.parseDouble(attrs.getValue(2));
            tempDouble = (long) (tempDouble * 1e2) / 1e2;
            rp.setForceZ(Double.toString(tempDouble));

        } else if (qName.equals("RobotSpeeds")) {

            tempDouble = Double.parseDouble(attrs.getValue(0));
            tempDouble = (long) (tempDouble * 1e2) / 1e2;
            rp.setSpeedX(Double.toString(tempDouble));

            tempDouble = Double.parseDouble(attrs.getValue(1));
            tempDouble = (long) (tempDouble * 1e2) / 1e2;
            rp.setSpeedY(Double.toString(tempDouble));

            tempDouble = Double.parseDouble(attrs.getValue(2));
            tempDouble = (long) (tempDouble * 1e2) / 1e2;
            rp.setSpeedZ(Double.toString(tempDouble));

        } else if (qName.equals("Cycle")) {

            rp.setIteration(attrs.getValue(0));

            tempDouble = Double.parseDouble(attrs.getValue(1));
            tempDouble = (long) (tempDouble * 1e2) / 1e2;

            rp.setElapsedTime(Double.toString(tempDouble));

        } else if (qName.equals("CurStep")) {
            rp.setCurrStep(attrs.getValue(2));

            tempDouble = Double.parseDouble(attrs.getValue(1));
            tempDouble = (long) (tempDouble * 1e2) / 1e2;

            rp.setCurrElapsedTime(Double.toString(tempDouble));
            rp.setIteration(attrs.getValue(0));

        } else if (qName.equals("PrevStep")) {

            rp.setPrevStep(attrs.getValue(1));

        } else if (qName.equals("NextStep")) {

            rp.setNextStep(attrs.getValue(1));
        }
    }

    public RobotProcess getRobotProcess() {
        return rp;
    }


    public void endElement(String namespaceURI, String localName, String qName)
            throws SAXException {

        if (localName.equals("Process"))
            inProcess = false;
    }

    public void characters(char ch[], int start, int length) {
    }


}