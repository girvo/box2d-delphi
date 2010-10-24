unit URevolute;

interface
{$I ..\..\Physics2D\Physics2D.inc}

uses
   UMain, UPhysics2DTypes, UPhysics2D, SysUtils;

type
   TRevolute = class(TTester)
   public
      m_joint: Tb2RevoluteJoint;

      constructor Create; override;
      procedure Step(var settings: TSettings; timeStep: Float); override;
      procedure Keyboard(key: Byte); override;
   end;

implementation

{ TRevolute }

constructor TRevolute.Create;
var
   ground: Tb2Body;
   bd: Tb2BodyDef;
   shape: Tb2EdgeShape;
   cshape: Tb2CircleShape;
   rjd: Tb2RevoluteJointDef;
   body: Tb2Body;
begin
   inherited;
   begin
      bd := Tb2BodyDef.Create;
      ground := m_world.CreateBody(bd);

      shape := Tb2EdgeShape.Create;
      shape.SetVertices(MakeVector(-40.0, 0.0), MakeVector(40.0, 0.0));
      ground.CreateFixture(shape, 0.0);
   end;

   begin
      cshape := Tb2CircleShape.Create;
      cshape.m_radius := 0.5;

      bd := Tb2BodyDef.Create;
      bd.bodyType := b2_dynamicBody;

      rjd := Tb2RevoluteJointDef.Create;

      SetValue(bd.position, 0.0, 20.0);
      body := m_world.CreateBody(bd);
      body.CreateFixture(cshape, 5.0);

      body.SetAngularVelocity(100.0);
      body.SetLinearVelocity(MakeVector(-8.0 * 100.0, 0.0));

      rjd.Initialize(ground, body, MakeVector(0.0, 12.0));
      rjd.motorSpeed := 1.0 * Pi;
      rjd.maxMotorTorque := 10000.0;
      rjd.enableMotor := False;
      rjd.lowerAngle := -0.25 * Pi;
      rjd.upperAngle := 0.5 * Pi;
      rjd.enableLimit := true;
      rjd.collideConnected := true;

      m_joint := Tb2RevoluteJoint(m_world.CreateJoint(rjd));
   end;
end;

procedure TRevolute.Keyboard(key: Byte);
begin
   case key of
      Ord('L'): m_joint.EnableLimit(not m_joint.IsLimitEnabled);
      Ord('S'): m_joint.EnableMotor(False);
   end;
end;

procedure TRevolute.Step(var settings: TSettings; timeStep: Float);
begin
   inherited;
   DrawText('Keys: (l) limits, (a) left, (s) off, (d) right');
end;

initialization
   RegisterTestEntry('Revolute', TRevolute);

end.
