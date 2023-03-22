package frc.Mechanisms;

public abstract class AbstractMechanism implements Runnable{

    private boolean runThread;

    private Thread thread;
    private int threadPeriod;

    public AbstractMechanism(int threadPeriod){
        this.threadPeriod = threadPeriod;
    }

    public void start(){
        if(!thread.isAlive() || thread == null){
            thread = new Thread(this);
            runThread = true;
            thread.start();
        }
    }

    public void kill(){
        runThread = false;
        System.out.println("Thread Killed");
    }

    public abstract void update();
    public abstract void smartDashboard();
    public abstract void smartDashboard_DEBUG();
    

    @Override
    public void run(){
        while(runThread == true){
            update();
            
            try
            {
                Thread.sleep(threadPeriod);
            }
            catch(InterruptedException e)
            {
                System.out.println("Interrupted: " + e.getMessage());   
            }
        }
    }
}