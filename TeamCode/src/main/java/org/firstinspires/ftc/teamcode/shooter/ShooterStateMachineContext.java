/*
 * ex: set ro:
 * DO NOT EDIT.
 * generated by smc (http://smc.sourceforge.net/)
 * from file : ShooterStateMachine.sm
 */

package org.firstinspires.ftc.teamcode.shooter;


public class ShooterStateMachineContext
    extends statemap.FSMContext
{
//---------------------------------------------------------------
// Member methods.
//

    public ShooterStateMachineContext(ShooterController owner)
    {
        this (owner, ShooterStateMachine.Idle);
    }

    public ShooterStateMachineContext(ShooterController owner, ShooterControllerState initState)
    {
        super (initState);

        _owner = owner;
    }

    @Override
    public void enterStartState()
    {
        getState().entry(this);
        return;
    }

    public void evActivate()
    {
        _transition = "evActivate";
        getState().evActivate(this);
        _transition = "";
        return;
    }

    public void evDeactivate()
    {
        _transition = "evDeactivate";
        getState().evDeactivate(this);
        _transition = "";
        return;
    }

    public void evInit()
    {
        _transition = "evInit";
        getState().evInit(this);
        _transition = "";
        return;
    }

    public void evShoot()
    {
        _transition = "evShoot";
        getState().evShoot(this);
        _transition = "";
        return;
    }

    public void evShooterReady()
    {
        _transition = "evShooterReady";
        getState().evShooterReady(this);
        _transition = "";
        return;
    }

    public void evTimeout()
    {
        _transition = "evTimeout";
        getState().evTimeout(this);
        _transition = "";
        return;
    }

    public ShooterControllerState getState()
        throws statemap.StateUndefinedException
    {
        if (_state == null)
        {
            throw(
                new statemap.StateUndefinedException());
        }

        return ((ShooterControllerState) _state);
    }

    protected ShooterController getOwner()
    {
        return (_owner);
    }

    public void setOwner(ShooterController owner)
    {
        if (owner == null)
        {
            throw (
                new NullPointerException(
                    "null owner"));
        }
        else
        {
            _owner = owner;
        }

        return;
    }

//---------------------------------------------------------------
// Member data.
//

    transient private ShooterController _owner;

    //-----------------------------------------------------------
    // Constants.
    //

    private static final long serialVersionUID = 1L;

//---------------------------------------------------------------
// Inner classes.
//

    public static abstract class ShooterControllerState
        extends statemap.State
    {
    //-----------------------------------------------------------
    // Member methods.
    //

        protected ShooterControllerState(String name, int id)
        {
            super (name, id);
        }

        protected void entry(ShooterStateMachineContext context) {}
        protected void exit(ShooterStateMachineContext context) {}

        protected void evActivate(ShooterStateMachineContext context)
        {
            Default(context);
        }

        protected void evDeactivate(ShooterStateMachineContext context)
        {
            Default(context);
        }

        protected void evInit(ShooterStateMachineContext context)
        {
            Default(context);
        }

        protected void evShoot(ShooterStateMachineContext context)
        {
            Default(context);
        }

        protected void evShooterReady(ShooterStateMachineContext context)
        {
            Default(context);
        }

        protected void evTimeout(ShooterStateMachineContext context)
        {
            Default(context);
        }

        protected void Default(ShooterStateMachineContext context)
        {
            throw (
                new statemap.TransitionUndefinedException(
                    "State: " +
                    context.getState().getName() +
                    ", Transition: " +
                    context.getTransition()));
        }

    //-----------------------------------------------------------
    // Member data.
    //

        //-------------------------------------------------------
    // Constants.
    //

        private static final long serialVersionUID = 1L;
    }

    /* package */ static abstract class ShooterStateMachine
    {
    //-----------------------------------------------------------
    // Member methods.
    //

    //-----------------------------------------------------------
    // Member data.
    //

        //-------------------------------------------------------
        // Constants.
        //

        public static final ShooterStateMachine_Idle Idle =
            new ShooterStateMachine_Idle("ShooterStateMachine.Idle", 0);
        public static final ShooterStateMachine_Init Init =
            new ShooterStateMachine_Init("ShooterStateMachine.Init", 1);
        public static final ShooterStateMachine_Deactivated Deactivated =
            new ShooterStateMachine_Deactivated("ShooterStateMachine.Deactivated", 2);
        public static final ShooterStateMachine_Activating Activating =
            new ShooterStateMachine_Activating("ShooterStateMachine.Activating", 3);
        public static final ShooterStateMachine_Reload Reload =
            new ShooterStateMachine_Reload("ShooterStateMachine.Reload", 4);
        public static final ShooterStateMachine_ReadyToShoot ReadyToShoot =
            new ShooterStateMachine_ReadyToShoot("ShooterStateMachine.ReadyToShoot", 5);
        public static final ShooterStateMachine_Shooting Shooting =
            new ShooterStateMachine_Shooting("ShooterStateMachine.Shooting", 6);
    }

    protected static class ShooterStateMachine_Default
        extends ShooterControllerState
    {
    //-----------------------------------------------------------
    // Member methods.
    //

        protected ShooterStateMachine_Default(String name, int id)
        {
            super (name, id);
        }

    //-----------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class ShooterStateMachine_Idle
        extends ShooterStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private ShooterStateMachine_Idle(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void evInit(ShooterStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(ShooterStateMachine.Init);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class ShooterStateMachine_Init
        extends ShooterStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private ShooterStateMachine_Init(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(ShooterStateMachineContext context)
            {
                ShooterController ctxt = context.getOwner();

            ctxt.retractLoader();
            return;
        }

        @Override
        protected void evTimeout(ShooterStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(ShooterStateMachine.Deactivated);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class ShooterStateMachine_Deactivated
        extends ShooterStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private ShooterStateMachine_Deactivated(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(ShooterStateMachineContext context)
            {
                ShooterController ctxt = context.getOwner();

            ctxt.setShooterActivation(false);
            return;
        }

        @Override
        protected void evActivate(ShooterStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(ShooterStateMachine.Activating);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class ShooterStateMachine_Activating
        extends ShooterStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private ShooterStateMachine_Activating(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(ShooterStateMachineContext context)
            {
                ShooterController ctxt = context.getOwner();

            ctxt.retractLoader();
            ctxt.setShooterActivation(true);
            return;
        }

        @Override
        protected void evDeactivate(ShooterStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(ShooterStateMachine.Deactivated);
            (context.getState()).entry(context);
            return;
        }

        @Override
        protected void evShooterReady(ShooterStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(ShooterStateMachine.ReadyToShoot);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class ShooterStateMachine_Reload
        extends ShooterStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private ShooterStateMachine_Reload(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(ShooterStateMachineContext context)
            {
                ShooterController ctxt = context.getOwner();

            ctxt.retractLoader();
            ctxt.startTimer(1000);
            return;
        }

        @Override
        protected void evDeactivate(ShooterStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(ShooterStateMachine.Deactivated);
            (context.getState()).entry(context);
            return;
        }

        @Override
        protected void evTimeout(ShooterStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(ShooterStateMachine.ReadyToShoot);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class ShooterStateMachine_ReadyToShoot
        extends ShooterStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private ShooterStateMachine_ReadyToShoot(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void evDeactivate(ShooterStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(ShooterStateMachine.Deactivated);
            (context.getState()).entry(context);
            return;
        }

        @Override
        protected void evShoot(ShooterStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(ShooterStateMachine.Shooting);
            (context.getState()).entry(context);
            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }

    private static final class ShooterStateMachine_Shooting
        extends ShooterStateMachine_Default
    {
    //-------------------------------------------------------
    // Member methods.
    //

        private ShooterStateMachine_Shooting(String name, int id)
        {
            super (name, id);
        }

        @Override
        protected void entry(ShooterStateMachineContext context)
            {
                ShooterController ctxt = context.getOwner();

            ctxt.shoot();
            ctxt.startTimer(1250);
            return;
        }

        @Override
        protected void evDeactivate(ShooterStateMachineContext context)
        {

            (context.getState()).exit(context);
            context.setState(ShooterStateMachine.Deactivated);
            (context.getState()).entry(context);
            return;
        }

        @Override
        protected void evTimeout(ShooterStateMachineContext context)
        {
            ShooterController ctxt = context.getOwner();

            if (ctxt.isShooterReady())
            {
                (context.getState()).exit(context);
                // No actions.
                context.setState(ShooterStateMachine.Reload);
                (context.getState()).entry(context);
            }
            else if (!ctxt.isShooterReady())
            {
                (context.getState()).exit(context);
                // No actions.
                context.setState(ShooterStateMachine.Activating);
                (context.getState()).entry(context);
            }            else
            {
                super.evTimeout(context);
            }

            return;
        }

    //-------------------------------------------------------
    // Member data.
    //

        //---------------------------------------------------
        // Constants.
        //

        private static final long serialVersionUID = 1L;
    }
}

/*
 * Local variables:
 *  buffer-read-only: t
 * End:
 */
