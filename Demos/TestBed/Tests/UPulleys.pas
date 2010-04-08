unit UPulleys;

interface
{$I ..\..\Physics2D\Physics2D.inc}

uses
   UMain, UPhysics2DTypes, UPhysics2D, SysUtils;

type
   TPulleys = class(TTester)
   public
      m_joint1: Tb2PulleyJoint;

      constructor Create; override;
      procedure Step(var settings: TSettings; timeStep: Float); override;
   end;

implementation

{ TPulleys }

constructor TPulleys.Create;
var
   ground: Tb2Body;
   bd: Tb2BodyDef;
   shape: Tb2PolygonShape;
   body1, body2: Tb2Body;
   pulleyDef: Tb2PulleyJointDef;
   b, y, L: Float;
begin
   inherited;
   begin
			bd := Tb2BodyDef.Create;
			ground := m_world.CreateBody(bd);

			shape := Tb2PolygonShape.Create;
			shape.SetAsEdge(MakeVector(-40.0, 0.0), MakeVector(40.0, 0.0));
			ground.CreateFixture(shape, 0.0);
   end;

   begin
      b := 4.0;
      y := 16.0;
      L := 12.0;

      shape := Tb2PolygonShape.Create;
      shape.SetAsBox(2, b);

      bd := Tb2BodyDef.Create;
      bd.bodyType := b2_dynamicBody;

      SetValue(bd.position, -10.0, y);
      body1 := m_world.CreateBody(bd, False);
      body1.CreateFixture(shape, 5.0, False);

      SetValue(bd.position, 10.0, y);
      body2 := m_world.CreateBody(bd);
      body2.CreateFixture(shape, 5.0);

      pulleyDef := Tb2PulleyJointDef.Create;
      pulleyDef.Initialize(body1, body2, MakeVector(-10.0, y + b + L),
         MakeVector(10.0, y + b + L), MakeVector(-10.0, y + b), MakeVector(10.0, y + b), 2.0);

      m_joint1 := Tb2PulleyJoint(m_world.CreateJoint(pulleyDef));
   end;
end;

procedure TPulleys.Step(var settings: TSettings; timeStep: Float);
var
   L: Float;
begin
   inherited;
   L := m_joint1.GetLength1 + m_joint1.GetRatio * m_joint1.GetLength2;
   DrawText(Format('L1 + %4.2f * L2 = %4.2f', [m_joint1.GetRatio, L]));
end;

initialization
   RegisterTestEntry('Pulleys', TPulleys);

end.
