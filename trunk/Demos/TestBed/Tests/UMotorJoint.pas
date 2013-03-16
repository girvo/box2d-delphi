unit UMotorJoint;

interface
{$I ..\..\Physics2D\Physics2D.inc}

uses
   UMain, UPhysics2DTypes, UPhysics2D, SysUtils;

type
   TMotorJoint = class(TTester)
   public
      m_joint: Tb2MotorJoint;
      m_time: PhysicsFloat;
      m_go: Boolean;

      constructor Create; override;
      procedure Step(var settings: TSettings; timeStep: PhysicsFloat); override;
      procedure Keyboard(key: Byte); override;
   end;

implementation

{ TMotorJoint }

constructor TMotorJoint.Create;
var
   bd: Tb2BodyDef;
   ground, body: Tb2Body;
   shape: Tb2EdgeShape;
   pshape: Tb2PolygonShape;
   fd: Tb2FixtureDef;
   mjd: Tb2MotorJointDef;
begin
   inherited;

   begin
      bd := Tb2BodyDef.Create;
      ground := m_world.CreateBody(bd);

      shape := Tb2EdgeShape.Create;
      shape.SetVertices(MakeVector(-20.0, 0.0), MakeVector(20.0, 0.0));

      fd := Tb2FixtureDef.Create;
      fd.shape := shape;

      ground.CreateFixture(fd);
   end;

   // Define motorized body
   begin
      bd := Tb2BodyDef.Create;
      bd.bodyType := b2_dynamicBody;
      SetValue(bd.position, 0.0, 8.0);
      body := m_world.CreateBody(bd);

      pshape := Tb2PolygonShape.Create;
      pshape.SetAsBox(2.0, 0.5);

      fd := Tb2FixtureDef.Create;
      fd.shape := pshape;
      fd.friction := 0.6;
      fd.density := 2.0;
      body.CreateFixture(fd);

      mjd := Tb2MotorJointDef.Create;
      mjd.Initialize(ground, body);
      mjd.maxForce := 1000.0;
      mjd.maxTorque := 1000.0;
      m_joint := Tb2MotorJoint(m_world.CreateJoint(mjd));
   end;

   m_go := False;
   m_time := 0.0;
end;

procedure TMotorJoint.Step(var settings: TSettings; timeStep: PhysicsFloat);
var
   linearOffset: TVector2;
   angularOffset: PhysicsFloat;
begin
   if m_go then
      m_time := m_time + timeStep;

   linearOffset.x := 6.0 * Sin(2.0 * m_time);
   linearOffset.y := 8.0 + 4.0 * Sin(1.0 * m_time);

   angularOffset := 4.0 * m_time;

   m_joint.SetLinearOffset(linearOffset);
   m_joint.SetAngularOffset(angularOffset);

   m_debugDraw.DrawPoint(linearOffset, 4.0, MakeColor(0.9, 0.9, 0.9));

   inherited;
   DrawText('Press ''S'' to pause.');
end;

procedure TMotorJoint.Keyboard(key: Byte);
begin
   case key of
      Ord('S'): m_go := not m_go;
   end;
end;

initialization
   RegisterTestEntry('Motor Joint', TMotorJoint);

end.
