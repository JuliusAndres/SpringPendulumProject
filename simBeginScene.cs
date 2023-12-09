using Godot;
using System;
using System.Runtime.CompilerServices;

public partial class simBeginScene : Node3D
{
	MeshInstance3D Anchor;
	MeshInstance3D Ball;
	SpringModel spring;
	Label KElabel;
	Label PElabel;
	Label TotalElabel;
	PendSim pend;

	double xA, yA, zA;	// coords of anchor
	float length0;	// natural length of pendulum
	float length;	// length of the pendulum
	//double angle;	// pendulum angle
	//double angleInitial;	//initial pendulum angle
	double time;	// Time variable
	double TotEnergy;
	Vector3 endA;	// vector for anchor
	Vector3 endB;	// vector for ball

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		GD.Print("Hello MEE381 in Godot");
		xA = 0.0; yA = 1.2; zA = 0.0;
		Anchor = GetNode<MeshInstance3D>("Anchor");
		Ball = GetNode<MeshInstance3D>("Ball");
		spring = GetNode<SpringModel>("SpringModel");
		endA = new Vector3((float)xA, (float)yA, (float)zA);
		Anchor.Position = endA;
		
		KElabel = GetNode<Label>("KElabel");
		PElabel = GetNode<Label>("PElabel");
		TotalElabel = GetNode<Label>("TotalElabel");

		pend = new PendSim();
		
		length0 = length = 0.9f;
		spring.GenMesh(0.05f, 0.015f, length, 6.0f, 62);

		// angleInitial = Mathf.DegToRad(60.0);
		// float angleF = (float)angleInitial;
		// pend.Angle = (double)angleInitial;

		// endB.X = endA.X + length*Mathf.Sin(angleF);
		// endB.Y = endA.Y - length*Mathf.Cos(angleF);
		// endB.Z = endA.Z;

		endB.X = endA.X + (float)pend.X;
		endB.Y = endA.Y + (float)pend.Y;
		endB.Z = endA.Z + (float)pend.Z; 

		PlacePendulum(endB);
		//PlacePendulum((float)angle);

		time = 0.0;
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		//float angleF = 1.0f*(float)Math.Sin(2.0 * time);
		//float angleA = (float)(0.4*time);
		//length = length0 + 0.3f * (float)Math.Cos(4.0 * time);

		TotEnergy = pend.KineticE + pend.PotentialE;
		
		//float angleA = 0.0f;		

		//float angleF = (float)pend.Angle;
		//KElabel.Text = angleF.ToString("0.00");

		KElabel.Text = pend.KineticE.ToString("0.00");
		PElabel.Text = pend.PotentialE.ToString("0.00");
		TotalElabel.Text = TotEnergy.ToString("0.00");
		
		//float hz = length*Mathf.Sin(angleF);

		// endB.X = endA.X + hz*Mathf.Cos(angleA);
		// endB.Y = endA.Y - length*Mathf.Cos(angleF);
		// endB.Z = endA.Z + hz*Mathf.Sin(angleA);
		
		endB.X = endA.X + (float)pend.X;
		endB.Y = endA.Y + (float)pend.Y;
		endB.Z = endA.Z + (float)pend.Z;

		PlacePendulum(endB);
		time += delta;
	}

    public override void _PhysicsProcess(double delta)
    {
        base._PhysicsProcess(delta);

		//Going to replace with RK4 Step here
		pend.StepRK4(time, delta);
    }

    private void PlacePendulum(Vector3 endBB)
	{
		spring.PlaceEndPoints(endA, endB);
		Ball.Position = endBB;
	}
}
