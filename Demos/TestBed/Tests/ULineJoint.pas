unit ULineJoint;

interface
{$I ..\..\Physics2D\Physics2D.inc}

uses
   UMain, UPhysics2DTypes, UPhysics2D, SysUtils;

type
   TLineJoint = class(TTester)
   public
      constructor Create; override;
   end;


implementation

{ TLineJoint }

constructor TLineJoint.Create;
var
   ground, body: Tb2Body;
   edge: Tb2EdgeShape;
   shape: Tb2PolygonShape;
   bd: Tb2BodyDef;
   jd: Tb2LineJointDef;
   axis: TVector2;
begin
   inherited;
   begin
      edge := Tb2EdgeShape.Create;
      edge.SetVertices(MakeVector(-40.0, 0.0), MakeVector(40.0, 0.0));

      bd := Tb2BodyDef.Create;
      ground := m_world.CreateBody(bd);
      ground.CreateFixture(edge, 0.0);
   end;

   begin
      shape := Tb2PolygonShape.Create;
      shape.SetAsBox(0.5, 2.0);

      bd := Tb2BodyDef.Create;
      bd.bodyType := b2_dynamicBody;
      SetValue(bd.position, 0.0, 7.0);
      body := m_world.CreateBody(bd);
      body.CreateFixture(shape, 1.0);

      jd := Tb2LineJointDef.Create;
      SetValue(axis, 2.0, 1.0);
      {$IFDEF OP_OVERLOAD}
      axis.Normalize;
      {$ELSE}
      Normalize(axis);
      {$ENDIF}
      jd.Initialize(ground, body, MakeVector(0.0, 8.5), axis);
      jd.motorSpeed := 0.0;
      jd.maxMotorForce := 100.0;
      jd.enableMotor := True;
      jd.lowerTranslation := -4.0;
      jd.upperTranslation := 4.0;
      jd.enableLimit := True;
      m_world.CreateJoint(jd);
   end;
end;

initialization
   RegisterTestEntry('Line Joint', TLineJoint);
end.
