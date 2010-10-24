unit UPhysics2DTypes;

{ box2D 2.1.0 translation

  ###  This unit is written based on Box2D maintained by Erin Catto (http://www.box2d.org)
  All type names follow the Delphi custom Txxx and xxx means the corresponding
  type in cpp source. This rule doesn't apply to the following types b2Vec2,
  b2Vec3, b2Mat22 and b2Mat33. Because I think these types are so primary that you
  may want to reuse them in other projects.

  ###  box2D 2.1.0 version is so different from 2.0.1 version that Erin nearly
  overwrote all codes. For more information I recommend reading the following page
  http://box2dflash.boristhebrave.com/docs/2.1a/updating

  ###  I translated version 2.0.1 without considering much of class encapsulation so
  you can find many m_xxx class memebers left out in public field. In this version
  I fixed all things and you should use Set/Get procedures to access these members.
  But for some members owning both Set and Get procedures I changed them to
  properties. For example I simplify SetUserData and GetUserData to such code
     Property UserData: Pointer read m_userData write m_userData;
  Another thing is that I changed some reference declaration(delphi doesn't support)
  to pointer to avoid unnecessary memory copy. You can check Tb2Body.GetTransform
  and Tb2DynamicTree.GetFatAABB for more details.

  ###  Another difference from version 2.0.1 is that I changed Tb2Contact
  declaration from class to record for effciency. Because class is created on
  heap and Tb2Contact is so frequently created and destroyed.

  ###  Because versions before Delphi 2006 don't support operator overloading, so
  I write two versions of all math operations for vector and matrix, etc. But
  later I found that the version without operator overloading runs faster.
  So if you want a better performance, DEFINE BETTER_PERFORMANCE in Physics2D.inc
  which will UNDEFINE OP_OVERLOAD even if you are using Delphi 2010.

  ###  This library supports three kinds of floats, Single(32bit), Double(64bit) and
  Extended(80bit). Double precision is always recommended because I will test all
  codes and demos using double precision. Using other float systems may cause some
  unpredictable exceptions.
       flags        EXTENDED_PRECISION        DOUBLE_PRECISION
     Extended               ON                     whatever
      Double(default)       OFF                       ON
      Single                OFF                       OFF
  There is also a flag SINGLE_PRECISION in the include file but it doesn't affect
  Float type definition.

  ###  Controllers are added as enhancement and can be flagged by CONTROLLERS.
  If you don't need them, please unflag to reduce code size.

  ###  If you want to do benchmark or something else, please flag COMPUTE_PHYSICSTIME.
  Time consumed by each step is updated and stored in Tb2World.GetPhysicsTime.

  ###  All assertions are ignored.

  Translator: Qianyuan Wang(ÍõÇ¬Ôª)
  Contact me: http://hi.baidu.com/wqyfavor
              wqyfavor@163.com
              QQ: 466798985
}

interface
{$I Physics2D.inc}

uses
   Math;

type
   Int8   = ShortInt;
   Int16  = SmallInt;
   Int32  = Integer;
   PInt32 = ^Int32;
   UInt8  = Byte;
   UInt16 = Word;
   UInt32 = Cardinal;
   PUInt16 = ^UInt16;

   TPointFloat = Single;
   TPointFloatArray = array of TPointFloat;

   PPointF = ^TPointF;
   TPointF = packed record
      x, y: TPointFloat;
      {$IFDEF OP_OVERLOAD}
      procedure SetZero; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure SetValue(x, y: TPointFloat); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      {$ENDIF}
   end;
   TPointsF = array of TPointF;

   // Float type
   PFloat = ^Float;
   {$IFDEF EXTENDED_PRECISION}
   Float = Extended;
   {$ELSE}
      {$IFDEF DOUBLE_PRECISION}
      Float = Double;
      {$ELSE}
      Float = Single;
      {$ENDIF}
   {$ENDIF}
   Float32 = Single;
   Float64 = Double;
   Float80 = Extended;

const
   {$IFDEF EXTENDED_PRECISION}
   FLT_EPSILON = 1.084202172485504E-19;
   FLT_MAX = MaxExtended;
   {$ELSE}
      {$IFDEF DOUBLE_PRECISION}
      FLT_EPSILON = 2.2204460492503131e-16;
      FLT_MAX = MaxDouble;
      {$ELSE}
      FLT_EPSILON = 1.192092896e-7;
      FLT_MAX = MaxSingle;
      {$ENDIF}
   {$ENDIF}

   function IsValid(f: Float): Boolean; {$IFNDEF OP_OVERLOAD}overload;{$ENDIF}

type
   PVector2 = ^TVector2;
   TVector2Arraied = array[0..1] of Float; // The same with TVector2
   TVector2 = record
      x, y: Float;

      {$IFDEF OP_OVERLOAD}
      function IsValid: Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      function Length: Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      function SqrLength: Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      function Normalize: Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure SetZero; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure SetNegative; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure SetValue(x, y: Float); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure SetLength(value: Float); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      class function From(const x, y: Float): TVector2; static; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      // Operators
      class operator Negative(const AValue: TVector2): TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class operator Add(const Left, Right: TVector2): TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class operator Subtract(const Left, Right: TVector2): TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      class operator Multiply(const Left: TVector2; const Right: Single): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class operator Multiply(const Left: TVector2; const Right: Double): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class operator Multiply(const Left: TVector2; const Right: Extended): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class operator Multiply(const Left: Single; const Right: TVector2): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class operator Multiply(const Left: Double; const Right: TVector2): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class operator Multiply(const Left: Extended; const Right: TVector2): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      class operator Divide(const Left: TVector2; const Right: Single): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class operator Divide(const Left: TVector2; const Right: Double): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class operator Divide(const Left: TVector2; const Right: Extended): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      procedure AddBy(const Operand: TVector2); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure SubtractBy(const Operand: TVector2); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure MultiplyBy(const Operand: Single); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure MultiplyBy(const Operand: Double); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure MultiplyBy(const Operand: Extended); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure DivideBy(const Operand: Single); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure DivideBy(const Operand: Double); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure DivideBy(const Operand: Extended); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      {$ENDIF}
   end;

   TVectorArray = array of TVector2;
   TVectorArray4 = array[0..3] of TVector2;

   TVector3 = record // Added from v2.1.0
      x, y, z: Float;

      {$IFDEF OP_OVERLOAD}
      function IsValid: Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      function Length: Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      function SqrLength: Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      function Normalize: Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure SetZero; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure SetNegative; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure SetValue(x, y, z: Float); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure SetLength(value: Float); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      class function From(const x, y, z: Float): TVector3; static; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      class operator Negative(const AValue: TVector3): TVector3; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class operator Add(const Left, Right: TVector3): TVector3; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class operator Subtract(const Left, Right: TVector3): TVector3; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      class operator Multiply(const Left: TVector3; const Right: Single): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class operator Multiply(const Left: TVector3; const Right: Double): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class operator Multiply(const Left: TVector3; const Right: Extended): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class operator Multiply(const Left: Single; const Right: TVector3): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class operator Multiply(const Left: Double; const Right: TVector3): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class operator Multiply(const Left: Extended; const Right: TVector3): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      class operator Divide(const Left: TVector3; const Right: Single): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class operator Divide(const Left: TVector3; const Right: Double): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class operator Divide(const Left: TVector3; const Right: Extended): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      procedure AddBy(const Operand: TVector3); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure SubtractBy(const Operand: TVector3); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure MultiplyBy(const Operand: Single); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure MultiplyBy(const Operand: Double); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure MultiplyBy(const Operand: Extended); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure DivideBy(const Operand: Single); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure DivideBy(const Operand: Double); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure DivideBy(const Operand: Extended); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      {$ENDIF}
   end;

   TMatrix22 = record
      col1, col2: TVector2;

      {$IFDEF OP_OVERLOAD}
      procedure SetIdentity; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure SetZero; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure SetValue(const _col1, _col2: TVector2); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      /// Initialize this matrix using an angle. This matrix becomes
	    /// an orthonormal rotation matrix.
      procedure SetValue(angle: Float); overload;

      function Invert: TMatrix22;
      function GetInverse: TMatrix22; {$IFDEF INLINE_AVAIL}inline;{$ENDIF} // The same with Invert, imported from v2.1.0
      function Solve(const b: TVector2): TVector2; // Solve A * x = b, where b is a column vector.

      // Operators
      class operator Negative(const AValue: TMatrix22): TMatrix22;
      class operator Add(const Left, Right: TMatrix22): TMatrix22;
      class operator Subtract(const Left, Right: TMatrix22): TMatrix22;
      {$ENDIF}
   end;

   TMatrix33 = record // Added from v2.1.0
      col1, col2, col3: TVector3;

      {$IFDEF OP_OVERLOAD}
      procedure SetIdentity; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure SetZero; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure SetValue(const _col1, _col2, _col3: TVector3); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Solve A * x = b, where b is a column vector. This is more efficient
	    /// than computing the inverse in one-shot cases.
      function Solve33(const b: TVector3): TVector3;
      /// Solve A * x = b, where b is a column vector. This is more efficient
      /// than computing the inverse in one-shot cases. Solve only the upper
      /// 2-by-2 matrix equation.
      function Solve22(const b: TVector2): TVector2;

      // Operators
      class operator Negative(const AValue: TMatrix33): TMatrix33;
      class operator Add(const Left, Right: TMatrix33): TMatrix33;
      class operator Subtract(const Left, Right: TMatrix33): TMatrix33;
      {$ENDIF}
   end;

   /// Renamed to Tb2Transform from v2.1.0
   /// A transform contains translation and rotation. It is used to represent
   /// the position and orientation of rigid bodies.
   Pb2Transform = ^Tb2Transform;
   Tb2Transform = record
      position: TVector2;
      R: TMatrix22;

      {$IFDEF OP_OVERLOAD}
      procedure SetIdentity; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure SetValue(const p: TVector2; angle: Float); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      function GetAngle: Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      class function From(const position: TVector2; const R: TMatrix22): Tb2Transform; static;
      {$ENDIF}
   end;

   /// This describes the motion of a body/shape for TOI computation.
   /// Shapes are defined with respect to the body origin, which may
   /// no coincide with the center of mass. However, to support dynamics
   /// we must interpolate the center of mass position.
   Tb2Sweep = record
      localCenter: TVector2; // local center of mass position
      c0, c: TVector2; // center world positions
      a0, a: Float; // world angles

      /// Fraction of the current time step in the range [0,1]
      /// c0 and a0 are the positions at alpha0.
      alpha0: Float;

      {$IFDEF OP_OVERLOAD}
      /// Get the interpolated transform at a specific time.
      /// @param beta is a factor in [0,1], where 0 indicates alpha0.
    	procedure GetTransform(var xf: Tb2Transform; beta: Float); /// Renamed to GetTransform from v2.1.0

      /// Advance the sweep forward, yielding a new initial state.
      /// @param alpha the new initial time.
      procedure Advance(alpha: Float);

	    /// Normalize the angles. Normalize an angle in radians to be between -pi and pi
	    procedure Normalize;
      {$ENDIF}
   end;

const
   b2Pnt2_Zero: TPointF = (X: 0.0; Y: 0.0);
   b2Vec2_Zero: TVector2 = (X: 0.0; Y: 0.0);
   b2Vec3_Zero: TVector3 = (X: 0.0; Y: 0.0; Z: 0.0);
   b2Mat22_identity: TMatrix22 = (col1: (X: 1.0; Y: 0.0); col2: (X: 0.0; Y: 1.0));
   b2Mat33_identity: TMatrix33 = (col1: (X: 1.0; Y: 0.0; Z: 0.0); col2: (X: 0.0; Y: 1.0; Z: 0.0); col3: (X: 0.0; Y: 0.0; Z: 1.0));
   b2XForm_identity: Tb2Transform = (position: (X: 0.0; Y: 0.0); R: (col1: (X: 1.0; Y: 0.0); col2: (X: 0.0; Y: 1.0)));
   //b2Transform_identity = b2XForm_identity; // imported from 2.1.0

const
   UInt8_MAX = $FF;
   UINT16_MAX = $FFFF;

   b2_maxManifoldPoints = 2;

   /// The maximum number of vertices on a convex polygon. You cannot increase
   /// this too much because b2BlockAllocator has a maximum object size.
   b2_maxPolygonVertices = 8;

   b2_maxProxies = 512; // this must be a power of two
   b2_maxPairs = 8 * b2_maxProxies;	// this must be a power of two

   /// This is used to fatten AABBs in the dynamic tree. This allows proxies
   /// to move by a small amount without triggering a tree adjustment.
   /// This is in meters.
   b2_aabbExtension =	0.1; // Added from v2.1.0
   /// This is used to fatten AABBs in the dynamic tree. This is used to predict
   /// the future position based on the current displacement.
   /// This is a dimensionless multiplier.
   b2_aabbMultiplier = 2.0; // Added from v2.1.0

   // Dynamics
   /// A small length used as a collision and constraint tolerance. Usually it is
   /// chosen to be numerically significant, but visually insignificant.
   b2_linearSlop = 0.005;	// 0.5 cm

   /// A small angle used as a collision and constraint tolerance. Usually it is
   /// chosen to be numerically significant, but visually insignificant.
   b2_angularSlop = 2.0 / 180.0 * Pi;			// 2 degrees

   /// The radius of the polygon/edge shape skin. This should not be modified. Making
   /// this smaller means polygons will have an insufficient buffer for continuous collision.
   /// Making it larger may create artifacts for vertex collision.
   b2_polygonRadius = (2.0 * b2_linearSlop); // Added from v2.1.0

   /// Maximum number of sub-steps per contact in continuous physics simulation.
   b2_maxSubSteps = 8;

   /// Maximum number of contacts to be handled to solve a TOI impact.
   b2_maxTOIContacts = 32;

   /// A velocity threshold for elastic collisions. Any collision with a relative linear
   /// velocity below this threshold will be treated as inelastic.
   b2_velocityThreshold = 1.0;		// 1 m/s

   /// The maximum linear position correction used when solving constraints. This helps to
   /// prevent overshoot.
   b2_maxLinearCorrection = 0.2;	// 20 cm

   /// The maximum angular position correction used when solving constraints. This helps to
   /// prevent overshoot.
   b2_maxAngularCorrection = 8.0 / 180.0 * Pi;			// 8 degrees

   /// The maximum linear velocity of a body. This limit is very large and is used
   /// to prevent numerical problems. You shouldn't need to adjust this.
   b2_maxTranslation = 2.0;
   b2_maxTranslationSquared = b2_maxTranslation * b2_maxTranslation;

   /// The maximum angular velocity of a body. This limit is very large and is used
   /// to prevent numerical problems. You shouldn't need to adjust this.
   b2_maxRotation = 0.5 * Pi;
   b2_maxRotationSquared = b2_maxRotation * b2_maxRotation;

   /// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
   /// that overlap is removed in one time step. However using values close to 1 often lead
   /// to overshoot.
   b2_contactBaumgarte = 0.2;

   // Sleep
   /// The time that a body must be still before it will go to sleep.
   b2_timeToSleep  = 0.5;									// half a second

   /// A body cannot sleep if its linear velocity is above this tolerance.
   b2_linearSleepTolerance = 0.01;		// 1 cm/s

   /// A body cannot sleep if its angular velocity is above this tolerance.
   b2_angularSleepTolerance = 2.0 / 180.0;		// 2 degrees/s

function MakePoint(x, y: TPointFloat): TPointF; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function MakeVector(x, y: Float): TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SetZero(var p: TPointF); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SetZero(var v: TVector2); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SetValue(var p: TPointF; ax, ay: TPointFloat); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SetValue(var v: TVector2; ax, ay: Float); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SetValue(var v: TVector3; ax, ay, az: Float); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

{$IFNDEF OP_OVERLOAD}
// For TVector2
function IsValid(const v: TVector2): Boolean; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function LengthVec(const v: TVector2): Float; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

function SqrLength(const v: TVector2): Float; overload;  {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function Normalize(var v: TVector2): Float; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SetNegative(var v: TVector2); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SetLengthVec(var v: TVector2; value: Float); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

function Negative(const AValue: TVector2): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function Add(const Left, Right: TVector2): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function Add(const p1, p2, p3: TVector2): TVector2; overload;
function Subtract(const Left, Right: TVector2): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

function Multiply(const Left: TVector2; const Right: Single): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function Multiply(const Left: TVector2; const Right: Double): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function Multiply(const Left: TVector2; const Right: Extended): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

function Divide(const Left: TVector2; const Right: Single): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function Divide(const Left: TVector2; const Right: Double): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function Divide(const Left: TVector2; const Right: Extended): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

procedure AddBy(var v: TVector2; const Operand: TVector2); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SubtractBy(var v: TVector2; const Operand: TVector2); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure MultiplyBy(var v: TVector2; const Operand: Single); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure MultiplyBy(var v: TVector2; const Operand: Double); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure MultiplyBy(var v: TVector2; const Operand: Extended); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure DivideBy(var v: TVector2; const Operand: Single); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure DivideBy(var v: TVector2; const Operand: Double); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure DivideBy(var v: TVector2; const Operand: Extended); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

// For TVector3
function IsValid(const v: TVector3): Boolean; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function LengthVec(const v: TVector3): Float; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

function SqrLength(const v: TVector3): Float; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function Normalize(var v: TVector3): Float; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SetNegative(var v: TVector3); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SetLengthVec(var v: TVector3; value: Float); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

function Negative(const AValue: TVector3): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function Add(const Left, Right: TVector3): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function Add(const p1, p2, p3: TVector3): TVector3; overload;
function Subtract(const Left, Right: TVector3): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

function Multiply(const Left: TVector3; const Right: Single): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function Multiply(const Left: TVector3; const Right: Double): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function Multiply(const Left: TVector3; const Right: Extended): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

function Divide(const Left: TVector3; const Right: Single): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function Divide(const Left: TVector3; const Right: Double): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function Divide(const Left: TVector3; const Right: Extended): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

procedure AddBy(var v: TVector3; const Operand: TVector3); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SubtractBy(var v: TVector3; const Operand: TVector3); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure MultiplyBy(var v: TVector3; const Operand: Single); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure MultiplyBy(var v: TVector3; const Operand: Double); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure MultiplyBy(var v: TVector3; const Operand: Extended); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure DivideBy(var v: TVector3; const Operand: Single); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure DivideBy(var v: TVector3; const Operand: Double); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure DivideBy(var v: TVector3; const Operand: Extended); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

// For TMatrix22
procedure SetIdentity(var m: TMatrix22); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SetZero(var m: TMatrix22); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SetValue(var m: TMatrix22; const _col1, _col2: TVector2); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SetValue(var m: TMatrix22; angle: Float); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

function Invert(const m: TMatrix22): TMatrix22;
function GetInverse(const m: TMatrix22): TMatrix22; {$IFDEF INLINE_AVAIL}inline;{$ENDIF} // The same with Invert, imported from v2.1.0
function Solve(const m: TMatrix22; const b: TVector2): TVector2; // Solve A * x = b, where b is a column vector.

function Negative(const AValue: TMatrix22): TMatrix22; overload;
function Add(const Left, Right: TMatrix22): TMatrix22; overload;
function Add(const m1, m2, m3: TMatrix22): TMatrix22; overload;
function Subtract(const Left, Right: TMatrix22): TMatrix22; overload;

// For TMatrix33
procedure SetIdentity(var m: TMatrix33); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SetZero(var m: TMatrix33); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SetValue(var m: TMatrix33; const _col1, _col2, _col3: TVector3); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function Solve33(var m: TMatrix33; const b: TVector3): TVector3;
function Solve22(var m: TMatrix33; const b: TVector2): TVector2;

function Negative(const AValue: TMatrix33): TMatrix33; overload;
function Add(const Left, Right: TMatrix33): TMatrix33; overload;
function Add(const m1, m2, m3: TMatrix33): TMatrix33; overload;
function Subtract(const Left, Right: TMatrix33): TMatrix33; overload;

// For T2bXForm
procedure SetIdentity(var xf: Tb2Transform); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SetValue(var xf: Tb2Transform; const p: TVector2; angle: Float); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function GetAngle(const xf: Tb2Transform): Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

// For Tb2Sweep
procedure GetTransform(const Sweep: Tb2Sweep; var xf: Tb2Transform; beta: Float);
procedure Advance(var Sweep: Tb2Sweep; alpha: Float);
procedure Normalize(var Sweep: Tb2Sweep); overload;

{$ENDIF}

function b2MixFriction(friction1, friction2: Float): Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2MixRestitution(restitution1, restitution2: Float): Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

function b2Max(const a, b: Float): Float; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2Max(const a, b: TVector2): TVector2; overload;
function b2Min(const a, b: Float): Float; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2Min(const a, b: TVector2): TVector2; overload;
function b2Max(const a, b: Int32): Int32; overload;
function b2Min(const a, b: Int32): Int32; overload;

procedure b2Swap(var a, b: Float); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure b2Swap(var a, b: Int32); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure b2Swap(var a, b: TVector2); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

function b2Clamp(const a, low, high: TVector2): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2Clamp(const a, low, high: Float): Float; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

function b2MiddlePoint(const a, b: TVector2): TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

function b2Dot(const a, b: TVector2): Float; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2Dot(const a, b: TVector3): Float; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2Cross(const a, b: TVector2): Float; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2Cross(const a: TVector2; s: Float): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2Cross(s: Float; const a: TVector2): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2Cross(const a, b: TVector3): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2Mul(const A: TMatrix22; const v: TVector2): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2MulT(const A: TMatrix22; const v: TVector2): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2Distance(const a, b: TVector2): Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2DistanceSquared(const a, b: TVector2): Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2Mul(const A, B: TMatrix22): TMatrix22; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2MulT(const A, B: TMatrix22): TMatrix22; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2Mul(const T: Tb2Transform; const v: TVector2): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2MulT(const T: Tb2Transform; const v: TVector2): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2Mul(const A: TMatrix33; const v: TVector3): TVector3; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2MulT(const A, B: Tb2Transform): Tb2Transform; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2Abs(const a: TVector2): TVector2; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function b2Abs(const a: TMatrix22): TMatrix22; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

{$IFDEF EXTENDED_PRECISION}
procedure SinCos(const Theta: Extended; var Sin, Cos: Extended);
{$ELSE}
{$IFDEF DOUBLE_PRECISION}
procedure SinCos(const Theta: Double; var Sin, Cos: Double);
{$ELSE}
procedure SinCos(const Theta: Single; var Sin, Cos: Single);
{$ENDIF}
{$ENDIF}

function RandomFloat: Float; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function RandomFloat(lo, hi: Float): Float; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

implementation

{$IFDEF EXTENDED_PRECISION}
procedure SinCos(const Theta: Extended; var Sin, Cos: Extended);
asm
   FLD  Theta
   FSINCOS
   FSTP TBYTE PTR [EDX]    // cosine
   FSTP TBYTE PTR [EAX]    // sine
end;
{$ELSE}
{$IFDEF DOUBLE_PRECISION}
procedure SinCos(const Theta: Double; var Sin, Cos: Double);
asm
   FLD  Theta
   FSINCOS
   FSTP QWORD PTR [EDX]    // cosine
   FSTP QWORD PTR [EAX]    // sine
end;
{$ELSE}
procedure SinCos(const Theta: Single; var Sin, Cos: Single);
asm
   FLD  Theta
   FSINCOS
   FSTP DWORD PTR [EDX]    // cosine
   FSTP DWORD PTR [EAX]    // sine
end;
{$ENDIF}
{$ENDIF}

function RandomFloat: Float;
begin
   Result := Random;
end;

function RandomFloat(lo, hi: Float): Float;
begin
   Result := (hi - lo) * Random + lo;
end;

function MakePoint(x, y: TPointFloat): TPointF;
begin
   Result.x := x;
   Result.y := y;
end;

function MakeVector(x, y: Float): TVector2;
begin
   Result.x := x;
   Result.y := y;
end;

procedure SetZero(var p: TPointF);
begin
   p := b2Pnt2_Zero;
end;

procedure SetZero(var v: TVector2);
begin
   v := b2Vec2_Zero;
end;

procedure SetValue(var p: TPointF; ax, ay: TPointFloat);
begin
   with p do
   begin
      x := ax;
      y := ay;
   end;
end;

procedure SetValue(var v: TVector2; ax, ay: Float);
begin
   with v do
   begin
      x := ax;
      y := ay;
   end;
end;

procedure SetValue(var v: TVector3; ax, ay, az: Float);
begin
   with v do
   begin
      x := ax;
      y := ay;
      z := az;
   end;
end;

{$IFNDEF OP_OVERLOAD}
// For TVector2
function IsValid(const v: TVector2): Boolean;
begin
   Result := UPhysics2DTypes.IsValid(v.x) and UPhysics2DTypes.IsValid(v.y);
end;

function LengthVec(const v: TVector2): Float;
begin
   with v do
      Result := Sqrt(x * x + y * y);
end;

function SqrLength(const v: TVector2): Float;
begin
   with v do
      Result := x * x + y * y;
end;

function Normalize(var v: TVector2): Float;
begin
   Result := LengthVec(v);
	 if Result < FLT_EPSILON then
   begin
      Result := 0.0;
      Exit;
   end;

   with v do
   begin
      x := x / Result;
      y := y / Result;
   end;
end;

procedure SetNegative(var v: TVector2);
begin
   with v do
   begin
      x := -x;
      y := -y;
   end;
end;

procedure SetLengthVec(var v: TVector2; value: Float);
var
   l: Float;
begin
   with v do
   begin
      l := value / LengthVec(v);
      x := x * l;
      y := y * l;
   end;
end;

function Negative(const AValue: TVector2): TVector2;
begin
   Result.x := -AValue.x;
   Result.y := -AValue.y;
end;

function Add(const Left, Right: TVector2): TVector2;
begin
   Result.x := Left.x + Right.x;
   Result.y := Left.y + Right.y;
end;

function Add(const p1, p2, p3: TVector2): TVector2;
begin
   Result.x := p1.x + p2.x + p3.x;
   Result.y := p1.y + p2.y + p3.y;
end;

function Subtract(const Left, Right: TVector2): TVector2;
begin
   Result.x := Left.x - Right.x;
   Result.y := Left.y - Right.y;
end;

function Multiply(const Left: TVector2; const Right: Single): TVector2;
begin
   Result.x := Left.x * Right;
   Result.y := Left.y * Right;
end;

function Multiply(const Left: TVector2; const Right: Double): TVector2;
begin
   Result.x := Left.x * Right;
   Result.y := Left.y * Right;
end;

function Multiply(const Left: TVector2; const Right: Extended): TVector2;
begin
   Result.x := Left.x * Right;
   Result.y := Left.y * Right;
end;

function Divide(const Left: TVector2; const Right: Single): TVector2;
begin
   Result.x := Left.x / Right;
   Result.y := Left.y / Right;
end;

function Divide(const Left: TVector2; const Right: Double): TVector2;
begin
   Result.x := Left.x / Right;
   Result.y := Left.y / Right;
end;

function Divide(const Left: TVector2; const Right: Extended): TVector2;
begin
   Result.x := Left.x / Right;
   Result.y := Left.y / Right;
end;

procedure AddBy(var v: TVector2; const Operand: TVector2);
begin
   with v do
   begin
      x := x + Operand.x;
      y := y + Operand.y;
   end;
end;

procedure SubtractBy(var v: TVector2; const Operand: TVector2);
begin
   with v do
   begin
      x := x - Operand.x;
      y := y - Operand.y;
   end;
end;

procedure MultiplyBy(var v: TVector2; const Operand: Single);
begin
   with v do
   begin
      x := x * Operand;
      y := y * Operand;
   end;
end;

procedure MultiplyBy(var v: TVector2; const Operand: Double);
begin
   with v do
   begin
      x := x * Operand;
      y := y * Operand;
   end;
end;

procedure MultiplyBy(var v: TVector2; const Operand: Extended);
begin
   with v do
   begin
      x := x * Operand;
      y := y * Operand;
   end;
end;

procedure DivideBy(var v: TVector2; const Operand: Single);
begin
   with v do
   begin
      x := x / Operand;
      y := y / Operand;
   end;
end;

procedure DivideBy(var v: TVector2; const Operand: Double);
begin
   with v do
   begin
      x := x / Operand;
      y := y / Operand;
   end;
end;

procedure DivideBy(var v: TVector2; const Operand: Extended);
begin
   with v do
   begin
      x := x / Operand;
      y := y / Operand;
   end;
end;

// For TVector3
function IsValid(const v: TVector3): Boolean;
begin
   Result := UPhysics2DTypes.IsValid(v.x) and UPhysics2DTypes.IsValid(v.y) and
      UPhysics2DTypes.IsValid(v.z);
end;

function LengthVec(const v: TVector3): Float;
begin
   with v do
      Result := Sqrt(x * x + y * y + z * z);
end;

function SqrLength(const v: TVector3): Float;
begin
   with v do
      Result := x * x + y * y + z * z;
end;

function Normalize(var v: TVector3): Float;
begin
   Result := LengthVec(v);
	 if Result < FLT_EPSILON then
   begin
      Result := 0.0;
      Exit;
   end;

   with v do
   begin
      x := x / Result;
      y := y / Result;
      z := z / Result;
   end;
end;

procedure SetNegative(var v: TVector3);
begin
   with v do
   begin
      x := -x;
      y := -y;
      z := -z;
   end;
end;

procedure SetLengthVec(var v: TVector3; value: Float);
var
   l: Float;
begin
   with v do
   begin
      l := value / LengthVec(v);
      x := x * l;
      y := y * l;
      z := z * l;
   end;
end;

function Negative(const AValue: TVector3): TVector3;
begin
   Result.x := -AValue.x;
   Result.y := -AValue.y;
   Result.z := -AValue.z;
end;

function Add(const Left, Right: TVector3): TVector3;
begin
   Result.x := Left.x + Right.x;
   Result.y := Left.y + Right.y;
   Result.z := Left.z + Right.z;
end;

function Add(const p1, p2, p3: TVector3): TVector3;
begin
   Result.x := p1.x + p2.x + p3.x;
   Result.y := p1.y + p2.y + p3.y;
   Result.z := p1.z + p2.z + p3.z;
end;

function Subtract(const Left, Right: TVector3): TVector3;
begin
   Result.x := Left.x - Right.x;
   Result.y := Left.y - Right.y;
   Result.z := Left.z - Right.z;
end;

function Multiply(const Left: TVector3; const Right: Single): TVector3;
begin
   Result.x := Left.x * Right;
   Result.y := Left.y * Right;
   Result.z := Left.z * Right;
end;

function Multiply(const Left: TVector3; const Right: Double): TVector3;
begin
   Result.x := Left.x * Right;
   Result.y := Left.y * Right;
   Result.z := Left.z * Right;
end;

function Multiply(const Left: TVector3; const Right: Extended): TVector3;
begin
   Result.x := Left.x * Right;
   Result.y := Left.y * Right;
   Result.z := Left.z * Right;
end;

function Divide(const Left: TVector3; const Right: Single): TVector3;
begin
   Result.x := Left.x / Right;
   Result.y := Left.y / Right;
   Result.z := Left.z / Right;
end;

function Divide(const Left: TVector3; const Right: Double): TVector3;
begin
   Result.x := Left.x / Right;
   Result.y := Left.y / Right;
   Result.z := Left.z / Right;
end;

function Divide(const Left: TVector3; const Right: Extended): TVector3;
begin
   Result.x := Left.x / Right;
   Result.y := Left.y / Right;
   Result.z := Left.z / Right;
end;

procedure AddBy(var v: TVector3; const Operand: TVector3);
begin
   with v do
   begin
      x := x + Operand.x;
      y := y + Operand.y;
      z := z + Operand.z;
   end;
end;

procedure SubtractBy(var v: TVector3; const Operand: TVector3);
begin
   with v do
   begin
      x := x - Operand.x;
      y := y - Operand.y;
      z := z - Operand.z;
   end;
end;

procedure MultiplyBy(var v: TVector3; const Operand: Single);
begin
   with v do
   begin
      x := x * Operand;
      y := y * Operand;
      z := z * Operand;
   end;
end;

procedure MultiplyBy(var v: TVector3; const Operand: Double);
begin
   with v do
   begin
      x := x * Operand;
      y := y * Operand;
      z := z * Operand;
   end;
end;

procedure MultiplyBy(var v: TVector3; const Operand: Extended);
begin
   with v do
   begin
      x := x * Operand;
      y := y * Operand;
      z := z * Operand;
   end;
end;

procedure DivideBy(var v: TVector3; const Operand: Single);
begin
   with v do
   begin
      x := x / Operand;
      y := y / Operand;
      z := z / Operand;
   end;
end;

procedure DivideBy(var v: TVector3; const Operand: Double);
begin
   with v do
   begin
      x := x / Operand;
      y := y / Operand;
      z := z / Operand;
   end;
end;

procedure DivideBy(var v: TVector3; const Operand: Extended);
begin
   with v do
   begin
      x := x / Operand;
      y := y / Operand;
      z := z / Operand;
   end;
end;

// For Matrix22
procedure SetIdentity(var m: TMatrix22);
begin
   m := b2Mat22_identity;
end;

procedure SetZero(var m: TMatrix22);
begin
   with m do
   begin
      col1.x := 0.0;
      col2.x := 0.0;
      col1.y := 0.0;
      col2.y := 0.0;
   end;
end;

procedure SetValue(var m: TMatrix22; const _col1, _col2: TVector2);
begin
   with m do
   begin
      col1 := _col1;
      col2 := _col2;
   end;
end;

procedure SetValue(var m: TMatrix22; angle: Float);
var
   c, s: Float;
begin
    SinCos(angle, s, c);

    with m do
    begin
       col1.x := c;
       col2.x := -s;
       col1.y := s;
       col2.y := c;
    end;
end;

function Invert(const m: TMatrix22): TMatrix22;
var
   a, b, c, d, det: Float;
begin
   with m do
   begin
      a := col1.x;
      b := col2.x;
      c := col1.y;
      d := col2.y;
   end;

   det := a * d - b * c;
   if det <> 0.0 then
      det := 1.0 / det;
   with Result do
   begin
      col1.x :=  det * d;
      col2.x := -det * b;
      col1.y := -det * c;
      col2.y :=  det * a;
   end;
end;

function GetInverse(const m: TMatrix22): TMatrix22;
begin
   Result := Invert(m);
end;

function Solve(const m: TMatrix22; const b: TVector2): TVector2;
var
   a11, a12, a21, a22, det: Float;
begin
   with m do
   begin
      a11 := col1.x;
      a12 := col2.x;
      a21 := col1.y;
      a22 := col2.y;
   end;
   det := a11 * a22 - a12 * a21;
   if det <> 0.0 then
      det := 1.0 / det;
   Result.x := det * (a22 * b.x - a12 * b.y);
   Result.y := det * (a11 * b.y - a21 * b.x);
end;

function Negative(const AValue: TMatrix22): TMatrix22;
begin
   with Result do
   begin
      col1.x := -AValue.col1.x;
      col1.y := -AValue.col1.y;
      col2.x := -AValue.col2.x;
      col2.y := -AValue.col2.y;
   end;
end;

function Add(const Left, Right: TMatrix22): TMatrix22;
begin
   with Result do
   begin
      col1.x := Left.col1.x + Right.col1.x;
      col1.y := Left.col1.y + Right.col1.y;
      col2.x := Left.col2.x + Right.col2.x;
      col2.y := Left.col2.y + Right.col2.y;
   end;
end;

function Add(const m1, m2, m3: TMatrix22): TMatrix22;
begin
   with Result do
   begin
      col1.x := m1.col1.x + m2.col1.x + m3.col1.x;
      col1.y := m1.col1.y + m2.col1.y + m3.col1.y;
      col2.x := m1.col2.x + m2.col2.x + m3.col2.x;
      col2.y := m1.col2.y + m2.col2.y + m3.col2.y;
   end;
end;

function Subtract(const Left, Right: TMatrix22): TMatrix22;
begin
   with Result do
   begin
      col1.x := Left.col1.x - Right.col1.x;
      col1.y := Left.col1.y - Right.col1.y;
      col2.x := Left.col2.x - Right.col2.x;
      col2.y := Left.col2.y - Right.col2.y;
   end;
end;

// For TMatrix33

procedure SetIdentity(var m: TMatrix33);
begin
   m := b2Mat33_identity;
end;

procedure SetZero(var m: TMatrix33);
begin
   with m do
   begin
      col1 := b2Vec3_Zero;
      col2 := b2Vec3_Zero;
      col3 := b2Vec3_Zero;
   end;
end;

procedure SetValue(var m: TMatrix33; const _col1, _col2, _col3: TVector3);
begin
   with m do
   begin
      col1 := _col1;
      col2 := _col2;
      col3 := _col3;
   end;
end;

function Solve33(var m: TMatrix33; const b: TVector3): TVector3;
var
   det: Float;
begin
   with m do
   begin
      det := b2Dot(col1, b2Cross(col2, col3));
      if det <> 0.0 then
         det := 1.0 / det;
      Result.x := det * b2Dot(b, b2Cross(col2, col3));
      Result.y := det * b2Dot(col1, b2Cross(b, col3));
      Result.z := det * b2Dot(col1, b2Cross(col2, b));
   end;
end;

function Solve22(var m: TMatrix33; const b: TVector2): TVector2;
var
   det: Float;
begin
   with m do
   begin
      det := col1.x * col2.y - col2.x * col1.y;
      if det <> 0.0 then
         det := 1.0 / det;
      Result.x := det * (col2.y * b.x - col2.x * b.y);
      Result.y := det * (col1.x * b.y - col1.y * b.x);
   end;
end;

function Negative(const AValue: TMatrix33): TMatrix33;
begin
   Result := AValue;
   with Result do
   begin
      SetNegative(col1);
      SetNegative(col2);
      SetNegative(col3);
   end;
end;

function Add(const Left, Right: TMatrix33): TMatrix33;
begin
   with Result do
   begin
      col1 := Add(Left.col1, Right.col1);
      col2 := Add(Left.col2, Right.col2);
      col3 := Add(Left.col3, Right.col3);
   end;
end;

function Add(const m1, m2, m3: TMatrix33): TMatrix33;
begin
   with Result do
   begin
      col1 := Add(m1.col1, m2.col1, m3.col1);
      col2 := Add(m1.col2, m2.col2, m3.col2);
      col3 := Add(m1.col3, m2.col3, m3.col3);
   end;
end;

function Subtract(const Left, Right: TMatrix33): TMatrix33;
begin
   with Result do
   begin
      col1 := Subtract(Left.col1, Right.col1);
      col2 := Subtract(Left.col2, Right.col2);
      col3 := Subtract(Left.col3, Right.col3);
   end;
end;

// For Tb2Transform

procedure SetIdentity(var xf: Tb2Transform);
begin
   with xf do
   begin
      position := b2Vec2_Zero;
      SetIdentity(R);
   end;
end;

procedure SetValue(var xf: Tb2Transform; const p: TVector2; angle: Float);
begin
   with xf do
   begin
      position := p;
      SetValue(R, angle);
   end;
end;

function GetAngle(const xf: Tb2Transform): Float;
begin
   with xf do
      Result := ArcTan2(R.col1.y, R.col1.x);
end;

// For Tb2Sweep

procedure GetTransform(const Sweep: Tb2Sweep; var xf: Tb2Transform; beta: Float);
var
   angle: Float;
begin
   with Sweep do
   begin
      xf.position := Add(Multiply(c0, 1.0 - beta), Multiply(c, beta));
      angle := (1.0 - beta) * a0 + beta * a;
      SetValue(xf.R, angle);

      // Shift to origin
      SubtractBy(xf.position, b2Mul(xf.R, localCenter));
   end;
end;

procedure Advance(var Sweep: Tb2Sweep; alpha: Float);
var
   beta: Float;
begin
   with Sweep do
   begin
      //b2Assert(alpha0 < 1.0f);
      beta := (alpha - alpha0) / (1.0 - alpha0);
      c0 := Add(Multiply(c0, 1.0 - beta), Multiply(c, beta));
      a0 := (1.0 - beta) * a0 + beta * a;
      alpha0 := alpha;
   end;
end;

procedure Normalize(var Sweep: Tb2Sweep);
const
   Pi2 = 2 * Pi;
var
   d: Float;
begin
   with Sweep do
   begin
      d :=  Pi2 * Floor(a0 / Pi2);
      a0 := a0 - d;
      a := a - d;
   end;
end;

{$ENDIF}

/// Friction mixing law. Feel free to customize this.
function b2MixFriction(friction1, friction2: Float): Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
begin
	 Result := Sqrt(friction1 * friction2);
end;

/// Restitution mixing law. Feel free to customize this.
function b2MixRestitution(restitution1, restitution2: Float): Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
begin
   if restitution1 > restitution2 then
      Result := restitution1
   else
      Result := restitution2;
end;

function b2Max(const a, b: Float): Float;
begin
   if a >= b then
      Result := a
   else
      Result := b;
end;

function b2Max(const a, b: TVector2): TVector2;
begin
   Result.x := b2Max(a.x, b.x);
   Result.y := b2Max(a.y, b.y);
end;

function b2Min(const a, b: Float): Float;
begin
   if a >= b then
      Result := b
   else
      Result := a;
end;

function b2Min(const a, b: TVector2): TVector2;
begin
   Result.x := b2Min(a.x, b.x);
   Result.y := b2Min(a.y, b.y);
end;

function b2Max(const a, b: Int32): Int32;
begin
   if a >= b then
      Result := a
   else
      Result := b;
end;

function b2Min(const a, b: Int32): Int32;
begin
   if a >= b then
      Result := b
   else
      Result := a;
end;

procedure b2Swap(var a, b: Float);
var
   tmp: Float;
begin
   tmp := a;
   a := b;
   b := tmp;
end;

procedure b2Swap(var a, b: Int32);
var
   tmp: Int32;
begin
   tmp := a;
   a := b;
   b := tmp;
end;

procedure b2Swap(var a, b: TVector2);
var
   tmp: TVector2;
begin
   tmp := a;
   a := b;
   b := tmp;
end;

function b2Clamp(const a, low, high: TVector2): TVector2;
begin
   Result := b2Max(low, b2Min(a, high));
end;

function b2Clamp(const a, low, high: Float): Float;
begin
   if a < low then
      Result := low
   else if a > high then
      Result := high
   else
      Result := a;
end;

function b2MiddlePoint(const a, b: TVector2): TVector2;
begin
   Result.x := (a.x + b.x) * 0.5;
   Result.y := (a.y + b.y) * 0.5;
end;

function IsValid(f: Float): Boolean;
begin
   Result := not (IsNan(f) or IsInfinite(f));
end;

/// Peform the dot product on two vectors.
function b2Dot(const a, b: TVector2): Float;
begin
   Result := a.x * b.x + a.y * b.y;
end;

function b2Dot(const a, b: TVector3): Float;
begin
   Result := a.x * b.x + a.y * b.y + a.z * b.z;
end;

/// Perform the cross product on two vectors. In 2D this produces a scalar.
function b2Cross(const a, b: TVector2): Float;
begin
	 Result := a.x * b.y - a.y * b.x;
end;

/// Perform the cross product on a vector and a scalar. In 2D this produces a vector.
function b2Cross(const a: TVector2; s: Float): TVector2;
begin
   Result.x := s * a.y;
   Result.y := -s * a.x;
end;

/// Perform the cross product on a scalar and a vector. In 2D this produces a vector.
function b2Cross(s: Float; const a: TVector2): TVector2;
begin
   Result.x := -s * a.y;
   Result.y := s * a.x;
end;

function b2Cross(const a, b: TVector3): TVector3;
begin
   Result.x := a.y * b.z - a.z * b.y;
   Result.y := a.z * b.x - a.x * b.z;
   Result.z := a.x * b.y - a.y * b.x;
end;

/// Multiply a matrix times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another.
function b2Mul(const A: TMatrix22; const v: TVector2): TVector2;
begin
   Result.x := A.col1.x * v.x + A.col2.x * v.y;
   Result.y := A.col1.y * v.x + A.col2.y * v.y;
end;

/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another (inverse transform).
function b2MulT(const A: TMatrix22; const v: TVector2): TVector2;
begin
   Result.x := b2Dot(v, A.col1);
   Result.y := b2Dot(v, A.col2);
end;

{$IFDEF OP_OVERLOAD}
function b2Distance(const a, b: TVector2): Float;
begin
   Result := (a - b).Length;
end;

function b2DistanceSquared(const a, b: TVector2): Float;
begin
   Result := (a - b).SqrLength;
end;
{$ELSE}
function b2Distance(const a, b: TVector2): Float;
begin
   Result := LengthVec(Subtract(a, b));
end;

function b2DistanceSquared(const a, b: TVector2): Float;
begin
   Result := SqrLength(Subtract(a, b));
end;
{$ENDIF}

// A * B
function b2Mul(const A, B: TMatrix22): TMatrix22;
begin
   Result.col1 := b2Mul(A, B.col1);
   Result.col2 := b2Mul(A, B.col2);
end;

// A^T * B
function b2MulT(const A, B: TMatrix22): TMatrix22;
begin
   with Result do
   begin
      col1.x := b2Dot(A.col1, B.col1);
      col1.y := b2Dot(A.col2, B.col1);
      col2.x := b2Dot(A.col1, B.col2);
      col2.y := b2Dot(A.col2, B.col2);
   end;
end;

{$IFDEF OP_OVERLOAD}
function b2Mul(const T: Tb2Transform; const v: TVector2): TVector2;
begin
   Result := T.position + b2Mul(T.R, v);
end;
{$ELSE}
function b2Mul(const T: Tb2Transform; const v: TVector2): TVector2;
var
   tmp: TVector2;
begin
   tmp := b2Mul(T.R, v);
   Result.x := T.position.x + tmp.x;
   Result.y := T.position.y + tmp.y;
end;
{$ENDIF}

{$IFDEF OP_OVERLOAD}
function b2MulT(const T: Tb2Transform; const v: TVector2): TVector2;
begin
   Result := b2MulT(T.R, v - T.position);
end;
{$ELSE}
function b2MulT(const T: Tb2Transform; const v: TVector2): TVector2;
var
   tmp: TVector2;
begin
   tmp := Subtract(v, T.position);
   Result := b2MulT(T.R, tmp);
end;
{$ENDIF}

{$IFDEF OP_OVERLOAD}
function b2Mul(const A: TMatrix33; const v: TVector3): TVector3;
begin
   Result := v.x * A.col1 + v.y * A.col2 + v.z * A.col3;
end;
{$ELSE}
function b2Mul(const A: TMatrix33; const v: TVector3): TVector3;
begin
   Result := Add(Multiply(A.col1, v.x), Multiply(A.col2, v.y), Multiply(A.col3, v.z));
end;
{$ENDIF}

function b2MulT(const A, B: Tb2Transform): Tb2Transform;
begin
   Result.R := b2MulT(A.R, B.R);
   {$IFDEF OP_OVERLOAD}
   Result.position := B.position - A.position;
   {$ELSE}
   Result.position := Subtract(B.position, A.position);
   {$ENDIF}
end;

function b2Abs(const a: TVector2): TVector2;
begin
   Result.x := Abs(a.x);
   Result.y := Abs(a.y);
end;

function b2Abs(const a: TMatrix22): TMatrix22;
begin
   Result.col1 := b2Abs(a.col1);
   Result.col2 := b2Abs(a.col2);
end;

{ TPointF }

{$IFDEF OP_OVERLOAD}

procedure TPointF.SetZero;
begin
   x := 0.0;
   y := 0.0;
end;

procedure TPointF.SetValue(x, y: TPointFloat);
begin
   Self.x := x;
   Self.y := y;
end;

{$ENDIF}

{ TVector2 }

{$IFDEF OP_OVERLOAD}

function TVector2.IsValid: Boolean;
begin
   Result := UPhysics2DTypes.IsValid(x) and UPhysics2DTypes.IsValid(y);
end;

function TVector2.Length: Float;
begin
   Result := Sqrt(x * x + y * y);
end;

function TVector2.Normalize: Float;
begin
   Result := Length();
	 if Result < FLT_EPSILON then
   begin
      Result := 0.0;
      Exit;
   end;
   x := x / Result;
   y := y / Result;
end;

procedure TVector2.SetZero;
begin
   x := 0.0;
   y := 0.0;
end;

procedure TVector2.SetNegative;
begin
   x := -x;
   y := -y;
end;

procedure TVector2.SetValue(x, y: Float);
begin
   Self.x := x;
   Self.y := y;
end;

procedure TVector2.SetLength(value: Float);
var
   l: Float;
begin
   l := value / Length;
   x := x * l;
   y := y * l;
end;

function TVector2.SqrLength: Float;
begin
   Result := x * x + y * y;
end;

class function TVector2.From(const x, y: Float): TVector2;
begin
   Result.x := x;
   Result.y := y;
end;

class operator TVector2.Negative(const AValue: TVector2): TVector2;
begin
   Result.x := -AValue.x;
   Result.y := -AValue.y;
end;

class operator TVector2.Add(const Left, Right: TVector2): TVector2;
begin
   Result.x := Left.x + Right.x;
   Result.y := Left.y + Right.y;
end;

class operator TVector2.Subtract(const Left, Right: TVector2): TVector2;
begin
   Result.x := Left.x - Right.x;
   Result.y := Left.y - Right.y;
end;

class operator TVector2.Multiply(const Left: TVector2; const Right: Single): TVector2;
begin
   Result.x := Left.x * Right;
   Result.y := Left.y * Right;
end;

class operator TVector2.Multiply(const Left: TVector2; const Right: Double): TVector2;
begin
   Result.x := Left.x * Right;
   Result.y := Left.y * Right;
end;

class operator TVector2.Multiply(const Left: TVector2; const Right: Extended): TVector2;
begin
   Result.x := Left.x * Right;
   Result.y := Left.y * Right;
end;

class operator TVector2.Multiply(const Left: Single; const Right: TVector2): TVector2;
begin
   Result.x := Left * Right.x;
   Result.y := Left * Right.y;
end;

class operator TVector2.Multiply(const Left: Double; const Right: TVector2): TVector2;
begin
   Result.x := Left * Right.x;
   Result.y := Left * Right.y;
end;

class operator TVector2.Multiply(const Left: Extended; const Right: TVector2): TVector2;
begin
   Result.x := Left * Right.x;
   Result.y := Left * Right.y;
end;

class operator TVector2.Divide(const Left: TVector2; const Right: Single): TVector2;
begin
   Result.x := Left.x / Right;
   Result.y := Left.y / Right;
end;

class operator TVector2.Divide(const Left: TVector2; const Right: Double): TVector2;
begin
   Result.x := Left.x / Right;
   Result.y := Left.y / Right;
end;

class operator TVector2.Divide(const Left: TVector2; const Right: Extended): TVector2;
begin
   Result.x := Left.x / Right;
   Result.y := Left.y / Right;
end;

procedure TVector2.AddBy(const Operand: TVector2);
begin
   x := x + Operand.x;
   y := y + Operand.y;
end;

procedure TVector2.SubtractBy(const Operand: TVector2);
begin
   x := x - Operand.x;
   y := y - Operand.y;
end;

procedure TVector2.MultiplyBy(const Operand: Single);
begin
   x := x * Operand;
   y := y * Operand;
end;

procedure TVector2.MultiplyBy(const Operand: Double);
begin
   x := x * Operand;
   y := y * Operand;
end;

procedure TVector2.MultiplyBy(const Operand: Extended);
begin
   x := x * Operand;
   y := y * Operand;
end;

procedure TVector2.DivideBy(const Operand: Single);
begin
   x := x / Operand;
   y := y / Operand;
end;

procedure TVector2.DivideBy(const Operand: Double);
begin
   x := x / Operand;
   y := y / Operand;
end;

procedure TVector2.DivideBy(const Operand: Extended);
begin
   x := x / Operand;
   y := y / Operand;
end;

{$ENDIF}

{ TMatrix22 }

{$IFDEF OP_OVERLOAD}
procedure TMatrix22.SetIdentity;
begin
   Self := b2Mat22_identity;
end;

procedure TMatrix22.SetZero;
begin
   col1 := b2Vec2_Zero;
   col2 := b2Vec2_Zero;
end;

procedure TMatrix22.SetValue(const _col1, _col2: TVector2);
begin
   col1 := _col1;
   col2 := _col2;
end;

procedure TMatrix22.SetValue(angle: Float);
var
   c, s: Float;
begin
    SinCos(angle, s, c);
		col1.x := c;
    col2.x := -s;
		col1.y := s;
    col2.y := c;
end;

function TMatrix22.Invert: TMatrix22;
var
   a, b, c, d, det: Float;
begin
   a := col1.x;
   b := col2.x;
   c := col1.y;
   d := col2.y;

   det := a * d - b * c;
   if det <> 0.0 then
      det := 1.0 / det;
   with Result do
   begin
      col1.x :=  det * d;
      col2.x := -det * b;
      col1.y := -det * c;
      col2.y :=  det * a;
   end;
end;

function TMatrix22.GetInverse: TMatrix22;
begin
   Result := Invert();
end;

function TMatrix22.Solve(const b: TVector2): TVector2;
var
   det: Float;
begin
   det := col1.x * col2.y - col2.x * col1.y;
   if det <> 0.0 then
      det := 1.0 / det;
   Result.x := det * (col2.y * b.x - col2.x * b.y);
   Result.y := det * (col1.x * b.y - col1.y * b.x);
end;

class operator TMatrix22.Negative(const AValue: TMatrix22): TMatrix22;
begin
   with Result do
   begin
      col1.SetNegative;
      col2.SetNegative;
   end;
end;

class operator TMatrix22.Add(const Left, Right: TMatrix22): TMatrix22;
begin
   with Result do
   begin
      col1 := Left.col1 + Right.col1;
      col2 := Left.col2 + Right.col2;
   end;
end;

class operator TMatrix22.Subtract(const Left, Right: TMatrix22): TMatrix22;
begin
   with Result do
   begin
      col1 := Left.col1 - Right.col1;
      col2 := Left.col2 - Right.col2;
   end;
end;

{$ENDIF}

{ TMatrix33}

{$IFDEF OP_OVERLOAD}

procedure TMatrix33.SetIdentity;
begin
   Self := b2Mat33_identity;
end;

procedure TMatrix33.SetZero;
begin
   col1 := b2Vec3_Zero;
   col2 := b2Vec3_Zero;
   col3 := b2Vec3_Zero;
end;

procedure TMatrix33.SetValue(const _col1, _col2, _col3: TVector3);
begin
   col1 := _col1;
   col2 := _col2;
   col3 := _col3;
end;

function TMatrix33.Solve33(const b: TVector3): TVector3;
var
   det: Float;
begin
   det := b2Dot(col1, b2Cross(col2, col3));
   if det <> 0.0 then
      det := 1.0 / det;
   Result.x := det * b2Dot(b, b2Cross(col2, col3));
   Result.y := det * b2Dot(col1, b2Cross(b, col3));
   Result.z := det * b2Dot(col1, b2Cross(col2, b));
end;

function TMatrix33.Solve22(const b: TVector2): TVector2;
var
   det: Float;
begin
   det := col1.x * col2.y - col2.x * col1.y;
	 if det <> 0.0 then
      det := 1.0 / det;
   Result.x := det * (col2.y * b.x - col2.x * b.y);
   Result.y := det * (col1.x * b.y - col1.y * b.x);
end;

class operator TMatrix33.Negative(const AValue: TMatrix33): TMatrix33;
begin
   with Result do
   begin
      col1.SetNegative;
      col2.SetNegative;
      col3.SetNegative;
   end;
end;

class operator TMatrix33.Add(const Left, Right: TMatrix33): TMatrix33;
begin
   with Result do
   begin
      col1 := Left.col1 + Right.col1;
      col2 := Left.col2 + Right.col2;
      col3 := Left.col3 + Right.col3;
   end;
end;

class operator TMatrix33.Subtract(const Left, Right: TMatrix33): TMatrix33;
begin
   with Result do
   begin
      col1 := Left.col1 - Right.col1;
      col2 := Left.col2 - Right.col2;
      col3 := Left.col3 - Right.col3;
   end;
end;

{$ENDIF}

{ Tb2Transform }
{$IFDEF OP_OVERLOAD}
procedure Tb2Transform.SetIdentity;
begin
   position.SetZero;
   R.SetIdentity;
end;

procedure Tb2Transform.SetValue(const p: TVector2; angle: Float);
begin
   position := p;
   R.SetValue(angle);
end;

function Tb2Transform.GetAngle: Float;
begin
   Result := ArcTan2(R.col1.y, R.col1.x);
end;

class function Tb2Transform.From(const position: TVector2; const R: TMatrix22): Tb2Transform;
begin
   Result.position := position;
   Result.R := R;
end;
{$ENDIF}

{ Tb2Sweep }
{$IFDEF OP_OVERLOAD}
procedure Tb2Sweep.GetTransform(var xf: Tb2Transform; beta: Float);
var
   angle: Float;
begin
   xf.position := (1.0 - beta) * c0 + beta * c;
   angle := (1.0 - beta) * a0 + beta * a;
   xf.R.SetValue(angle);

   // Shift to origin
   xf.position.SubtractBy(b2Mul(xf.R, localCenter));
end;

procedure Tb2Sweep.Advance(alpha: Float);
var
   beta: Float;
begin
   //b2Assert(alpha0 < 1.0f);
   beta := (alpha - alpha0) / (1.0 - alpha0);
   c0 := (1.0 - beta) * c0 + beta * c;
   a0 := (1.0 - beta) * a0 + beta * a;
   alpha0 := alpha;
end;

procedure Tb2Sweep.Normalize;
const
   Pi2 = 2 * Pi;
var
   d: Float;
begin
   d :=  Pi2 * Floor(a0 / Pi2);
   a0 := a0 - d;
   a := a - d;
end;
{$ENDIF}

{ TVector3 }

{$IFDEF OP_OVERLOAD}
function TVector3.IsValid: Boolean;
begin
   Result := UPhysics2DTypes.IsValid(x) and UPhysics2DTypes.IsValid(y) and
      UPhysics2DTypes.IsValid(z);
end;

function TVector3.Length: Float;
begin
   Result := Sqrt(x * x + y * y + z * z);
end;

function TVector3.Normalize: Float;
begin
   Result := Length();
	 if Result < FLT_EPSILON then
   begin
      Result := 0.0;
      Exit;
   end;
   x := x / Result;
   y := y / Result;
   z := z / Result;
end;

procedure TVector3.SetZero;
begin
   x := 0.0;
   y := 0.0;
   z := 0.0;
end;

procedure TVector3.SetNegative;
begin
   x := -x;
   y := -y;
   z := -z;
end;

procedure TVector3.SetValue(x, y, z: Float);
begin
   Self.x := x;
   Self.y := y;
   Self.z := z;
end;

procedure TVector3.SetLength(value: Float);
var
   l: Float;
begin
   l := value / Length;
   x := x * l;
   y := y * l;
   z := z * l;
end;

function TVector3.SqrLength: Float;
begin
   Result := x * x + y * y + z * z;
end;

class function TVector3.From(const x, y, z: Float): TVector3;
begin
   Result.x := x;
   Result.y := y;
   Result.z := z;
end;

class operator TVector3.Negative(const AValue: TVector3): TVector3;
begin
   Result.x := -AValue.x;
   Result.y := -AValue.y;
   Result.z := -AValue.z;
end;

class operator TVector3.Add(const Left, Right: TVector3): TVector3;
begin
   Result.x := Left.x + Right.x;
   Result.y := Left.y + Right.y;
   Result.z := Left.z + Right.z;
end;

class operator TVector3.Subtract(const Left, Right: TVector3): TVector3;
begin
   Result.x := Left.x - Right.x;
   Result.y := Left.y - Right.y;
   Result.z := Left.z - Right.z;
end;

class operator TVector3.Multiply(const Left: TVector3; const Right: Single): TVector3;
begin
   Result.x := Left.x * Right;
   Result.y := Left.y * Right;
   Result.z := Left.z * Right;
end;

class operator TVector3.Multiply(const Left: TVector3; const Right: Double): TVector3;
begin
   Result.x := Left.x * Right;
   Result.y := Left.y * Right;
   Result.z := Left.z * Right;
end;

class operator TVector3.Multiply(const Left: TVector3; const Right: Extended): TVector3;
begin
   Result.x := Left.x * Right;
   Result.y := Left.y * Right;
   Result.z := Left.z * Right;
end;

class operator TVector3.Multiply(const Left: Single; const Right: TVector3): TVector3;
begin
   Result.x := Left * Right.x;
   Result.y := Left * Right.y;
   Result.z := Left * Right.z;
end;

class operator TVector3.Multiply(const Left: Double; const Right: TVector3): TVector3;
begin
   Result.x := Left * Right.x;
   Result.y := Left * Right.y;
   Result.z := Left * Right.z;
end;

class operator TVector3.Multiply(const Left: Extended; const Right: TVector3): TVector3;
begin
   Result.x := Left * Right.x;
   Result.y := Left * Right.y;
   Result.z := Left * Right.z;
end;

class operator TVector3.Divide(const Left: TVector3; const Right: Single): TVector3;
begin
   Result.x := Left.x / Right;
   Result.y := Left.y / Right;
   Result.z := Left.z / Right;
end;

class operator TVector3.Divide(const Left: TVector3; const Right: Double): TVector3;
begin
   Result.x := Left.x / Right;
   Result.y := Left.y / Right;
   Result.z := Left.z / Right;
end;

class operator TVector3.Divide(const Left: TVector3; const Right: Extended): TVector3;
begin
   Result.x := Left.x / Right;
   Result.y := Left.y / Right;
   Result.z := Left.z / Right;
end;

procedure TVector3.AddBy(const Operand: TVector3);
begin
   x := x + Operand.x;
   y := y + Operand.y;
   z := z + Operand.z;
end;

procedure TVector3.SubtractBy(const Operand: TVector3);
begin
   x := x - Operand.x;
   y := y - Operand.y;
   z := z - Operand.z;
end;

procedure TVector3.MultiplyBy(const Operand: Single);
begin
   x := x * Operand;
   y := y * Operand;
   z := z * Operand;
end;

procedure TVector3.MultiplyBy(const Operand: Double);
begin
   x := x * Operand;
   y := y * Operand;
   z := z * Operand;
end;

procedure TVector3.MultiplyBy(const Operand: Extended);
begin
   x := x * Operand;
   y := y * Operand;
   z := z * Operand;
end;

procedure TVector3.DivideBy(const Operand: Single);
begin
   x := x / Operand;
   y := y / Operand;
   z := z / Operand;
end;

procedure TVector3.DivideBy(const Operand: Double);
begin
   x := x / Operand;
   y := y / Operand;
   z := z / Operand;
end;

procedure TVector3.DivideBy(const Operand: Extended);
begin
   x := x / Operand;
   y := y / Operand;
   z := z / Operand;
end;
{$ENDIF}

end.
