/*
    This file is part of Magnum.

    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019,
        2020, 2021, 2022, 2023, 2024, 2025, 2026
             — Vladimír Vondruš <mosra@centrum.cz>
        2026 — Igal Alkon <igal@alkontek.com>

    This is free and unencumbered software released into the public domain.

    Anyone is free to copy, modify, publish, use, compile, sell, or distribute
    this software, either in source code form or as a compiled binary, for any
    purpose, commercial or non-commercial, and by any means.

    In jurisdictions that recognize copyright laws, the author or authors of
    this software dedicate any and all copyright interest in the software to
    the public domain. We make this dedication for the benefit of the public
    at large and to the detriment of our heirs and successors. We intend this
    dedication to be an overt act of relinquishment in perpetuity of all
    present and future rights to this software under copyright law.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Pointer.h>

#include <Magnum/Timeline.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Time.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/Transform.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/UVSphere.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/Shaders/PhongGL.h>
#include <Magnum/Trade/MeshData.h>

/* Jolt requires <Jolt/Jolt.h> BEFORE any other Jolt header.
We include the full definitions here because the converters return Jolt types by value. */
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>

#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Quaternion.h>

#include "JoltIntegration/Integration.h"
#include "JoltIntegration/DebugDraw.h"

namespace Magnum { namespace Examples {

using namespace Math::Literals;

typedef SceneGraph::Object<SceneGraph::MatrixTransformation3D> Object3D;
typedef SceneGraph::Scene<SceneGraph::MatrixTransformation3D> Scene3D;

struct InstanceData {
     Matrix4 transformationMatrix;
     Matrix3x3 normalMatrix;
     Color3 color;
};

struct AllCollideBroadPhaseLayerInterface : JPH::BroadPhaseLayerInterface {
    JPH::uint GetNumBroadPhaseLayers() const override { return 1; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer) const override { return JPH::BroadPhaseLayer(0); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char * GetBroadPhaseLayerName(JPH::BroadPhaseLayer) const override { return "All"; }
#endif
};

struct AllCollideObjectVsBroadPhaseLayerFilter : JPH::ObjectVsBroadPhaseLayerFilter {
    bool ShouldCollide(JPH::ObjectLayer inLayer, JPH::BroadPhaseLayer inBroadPhaseLayer) const override { return true; }
};

struct AllCollideObjectLayerPairFilter : JPH::ObjectLayerPairFilter {
    bool ShouldCollide(JPH::ObjectLayer, JPH::ObjectLayer) const override {
        return true;
    }
};

class JoltExample: public Platform::Application {
public:
    ~JoltExample();
    explicit JoltExample(const Arguments& arguments);

private:
    void drawEvent() override;
    void keyPressEvent(KeyEvent& event) override;
    void pointerPressEvent(PointerEvent& event) override;

    GL::Mesh _box{NoCreate}, _sphere{NoCreate};
    GL::Buffer _boxInstanceBuffer{NoCreate}, _sphereInstanceBuffer{NoCreate};
    Shaders::PhongGL _shader{NoCreate};
    Containers::Array<InstanceData> _boxInstanceData, _sphereInstanceData;

    Scene3D _scene;
    SceneGraph::Camera3D* _camera;
    SceneGraph::DrawableGroup3D _drawables;
    Timeline _timeline;

    Object3D *_cameraRig, *_cameraObject;

    /* Jolt */
    JPH::JobSystemThreadPool _jobSystem{};

    Containers::Pointer<JPH::TempAllocatorImpl> _tmpAllocator;
    Containers::Pointer<JPH::PhysicsSystem> _physicsSystem;
    Containers::Pointer<JPH::Shape> _boxShape;
    Containers::Pointer<JPH::Shape> _sphereShape;
    Containers::Pointer<JPH::Shape> _groundShape;

    AllCollideBroadPhaseLayerInterface _broadPhaseLayerInterface;
    AllCollideObjectVsBroadPhaseLayerFilter _objectVsBroadPhaseLayerFilter;
    AllCollideObjectLayerPairFilter _objectLayerPairFilter;

    Containers::Pointer<JoltIntegration::DebugDraw> _debugDraw;

    bool _drawCubes{true}, _drawDebug{true}, _shootBox{true};
};

class ColoredDrawable: public SceneGraph::Drawable3D {
    public:
        explicit ColoredDrawable(Object3D& object, Containers::Array<InstanceData>& instanceData, const Color3& color,
            const Matrix4& primitiveTransformation, SceneGraph::DrawableGroup3D& drawables):
            SceneGraph::Drawable3D{object, &drawables}, _instanceData(instanceData), _color{color},
            _primitiveTransformation{primitiveTransformation} {}

    private:
        void draw(const Matrix4& transformation, SceneGraph::Camera3D&) override {
            const Matrix4 t = transformation*_primitiveTransformation;
            arrayAppend(_instanceData, InPlaceInit, t, t.normalMatrix(), _color);
        }

        Containers::Array<InstanceData>& _instanceData;
        Color3 _color;
        Matrix4 _primitiveTransformation;
};

    class RigidBody: public Object3D {
    public:
        RigidBody(Object3D* parent, Float mass, const JPH::RefConst<JPH::Shape>& shape, JPH::PhysicsSystem& physicsSystem);

        ~RigidBody() override;

        JPH::BodyInterface& bodyInterface() const { return _physicsSystem.GetBodyInterface(); }
        JPH::BodyID bodyID() const { return _body->GetID(); }

        void syncPose() const;        // Magnum → Jolt
        void syncFromPhysics(); // Jolt → Magnum

    private:
        JPH::PhysicsSystem& _physicsSystem;
        JPH::Body* _body;
        bool _broken = false;
    };

RigidBody::RigidBody(Object3D* parent, const Float mass, const JPH::RefConst<JPH::Shape>& shape, JPH::PhysicsSystem& physicsSystem):
    Object3D{parent}, _physicsSystem{physicsSystem}
{
    const JPH::EMotionType motionType = (mass == 0.0f) ? JPH::EMotionType::Static : JPH::EMotionType::Dynamic;
    JPH::BodyCreationSettings settings(
        shape,
        JPH::Vec3::sZero(),
        JPH::Quat::sIdentity(),
        motionType,
        static_cast<JPH::ObjectLayer>(0)
    );

    settings.mFriction = 0.075f;
    settings.mRestitution = 0.35f;

    if(mass > 0.0f) {
        auto [mMass, mInertia] = shape->GetMassProperties();
        if (const float volume = mMass; volume > 0.0f) {
            const float density = mass / volume;
            settings.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
            settings.mMassPropertiesOverride.mMass = mass;
            settings.mMassPropertiesOverride.mInertia = mInertia * density;
        }
    }

    auto& bodyInterface = _physicsSystem.GetBodyInterface();
    _body = bodyInterface.CreateBody(settings);
    bodyInterface.AddBody(_body->GetID(), JPH::EActivation::Activate);

    syncPose();  // initial sync after creation
}

RigidBody::~RigidBody() {
    auto& bodyInterface = _physicsSystem.GetBodyInterface();
    bodyInterface.RemoveBody(_body->GetID());
    bodyInterface.DestroyBody(_body->GetID());
}

void RigidBody::syncPose() const
{
    const Matrix4& transformation = transformationMatrix();

    /* position */
    const Vector3 position = transformation.translation();
    const JPH::Vec3 positionJ = Math::Implementation::VectorConverter<3, Float, JPH::Vec3>::to(position);

    /* rotation */
    const Matrix3x3 rotationMatrix = transformation.rotationScaling();
    const Quaternion rotation = Quaternion::fromMatrix(rotationMatrix);
    const JPH::Quat rotationJ = Math::Implementation::QuaternionConverter<Float, JPH::Quat>::to(rotation);

    bodyInterface().SetPositionAndRotation(_body->GetID(), positionJ, rotationJ, JPH::EActivation::DontActivate);
}

void RigidBody::syncFromPhysics()
{
    JPH::Vec3 positionJ{};
    JPH::Quat rotationJ{};
    bodyInterface().GetPositionAndRotation(_body->GetID(), positionJ, rotationJ);

    if (positionJ.IsNaN() || rotationJ.IsNaN()) {
        if (!_broken) {
            Warning{} << "JoltIntegration::RigidBody: Jolt reported NaN transform for"
                      << this << Debug::nospace << ", ignoring this frame";
            _broken = true;
        }
        return;
    }

    _broken = false;

    const Vector3 position = Math::Implementation::VectorConverter<3, Float, JPH::Vec3>::from(positionJ);
    const Quaternion rotation = Math::Implementation::QuaternionConverter<Float, JPH::Quat>::from(rotationJ);

    resetTransformation()
        .rotate(rotation)
        .translate(position);
}

JoltExample::JoltExample(const Arguments& arguments) :
    Platform::Application(arguments, NoCreate)
{
    /* Try 8x MSAA, fall back to zero samples if not possible. Enable only 2x
       MSAA if we have enough DPI. */
    {
        const Vector2 dpiScaling = this->dpiScaling({});
        Configuration conf;
        conf.setTitle("Magnum JoltPhysics Integration Example")
            .setSize(conf.size(), dpiScaling);
        GLConfiguration glConf;
        glConf.setSampleCount(dpiScaling.max() < 2.0f ? 8 : 2);
        if (!tryCreate(conf, glConf))
            create(conf, glConf.setSampleCount(0));
    }

    /* Camera setup */
    (*(_cameraRig = new Object3D{&_scene}))
        .translate(Vector3::yAxis(3.0f))
        .rotateY(40.0_degf);
    (*(_cameraObject = new Object3D{_cameraRig}))
        .translate(Vector3::zAxis(20.0f))
        .rotateX(-25.0_degf);
    (_camera = new SceneGraph::Camera3D(*_cameraObject))
        ->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
        .setProjectionMatrix(Matrix4::perspectiveProjection(35.0_degf, 1.0f, 0.001f, 100.0f))
        .setViewport(GL::defaultFramebuffer.viewport().size());

    /* Create an instanced shader */
    _shader = Shaders::PhongGL{
        Shaders::PhongGL::Configuration{}
        .setFlags(Shaders::PhongGL::Flag::VertexColor |
            Shaders::PhongGL::Flag::InstancedTransformation)
    };
    _shader.setAmbientColor(0x111111_rgbf)
           .setSpecularColor(0x330000_rgbf)
           .setLightPositions({{10.0f, 15.0f, 5.0f, 0.0f}});

    /* Box and sphere mesh, with an (initially empty) instance buffer */
    _box = MeshTools::compile(Primitives::cubeSolid());
    _sphere = MeshTools::compile(Primitives::uvSphereSolid(16, 32));
    _boxInstanceBuffer = GL::Buffer{};
    _sphereInstanceBuffer = GL::Buffer{};
    _box.addVertexBufferInstanced(_boxInstanceBuffer, 1, 0,
                                  Shaders::PhongGL::TransformationMatrix{},
                                  Shaders::PhongGL::NormalMatrix{},
                                  Shaders::PhongGL::Color3{});
    _sphere.addVertexBufferInstanced(_sphereInstanceBuffer, 1, 0,
                                     Shaders::PhongGL::TransformationMatrix{},
                                     Shaders::PhongGL::NormalMatrix{},
                                     Shaders::PhongGL::Color3{});

    /* Setup the renderer so we can draw the debug lines on top */
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
    GL::Renderer::enable(GL::Renderer::Feature::PolygonOffsetFill);
    GL::Renderer::setPolygonOffset(2.0f, 0.5f);

    /* Jolt setup */
    JPH::RegisterDefaultAllocator();
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();

    _debugDraw = Containers::Pointer{new JoltIntegration::DebugDraw{}};
    _debugDraw->setMode(
        static_cast<UnsignedInt>(
            JoltIntegration::DebugDraw::Mode::DrawShape
            | JoltIntegration::DebugDraw::Mode::DrawShapeWireframe
            // | JoltIntegration::DebugDraw::Mode::DrawVelocity
            // | JoltIntegration::DebugDraw::Mode::DrawBoundingBox
        )
    );

    _tmpAllocator = Containers::Pointer{new JPH::TempAllocatorImpl(10 * 1024 * 1024)};

    _jobSystem.Init(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers,
        std::thread::hardware_concurrency() - 1);

    _physicsSystem = Containers::Pointer{new JPH::PhysicsSystem{}};
    _physicsSystem->Init(
        4096,           /* maxBodies */
        64,        /* bodyMutexes (power-of-2) */
        16384,        /* maxBodyPairs */
        8192,  /* maxContactConstraints */
        _broadPhaseLayerInterface,
        _objectVsBroadPhaseLayerFilter,
        _objectLayerPairFilter
    );
    _physicsSystem->SetGravity(JPH::Vec3{0.0f, -9.81f, 0.0f});

    /* Shapes */
    _boxShape = Containers::Pointer<JPH::Shape>{new JPH::BoxShape(JPH::Vec3{0.5f, 0.5f, 0.5f})};
    _sphereShape = Containers::Pointer<JPH::Shape>{new JPH::SphereShape{0.25f}};
    _groundShape = Containers::Pointer<JPH::Shape>{new JPH::BoxShape(JPH::Vec3{4.0f, 0.5f, 4.0f})};

    /* Create the ground */
    Object3D* ground = new RigidBody{&_scene, 0.0f, JPH::RefConst{_groundShape.get()}, *_physicsSystem};
    new ColoredDrawable{
        *ground, _boxInstanceData, 0xffffff_rgbf,
        Matrix4::scaling({4.0f, 0.5f, 4.0f}), _drawables
    };

    /* Create boxes with random colors */
    Deg hue = 42.0_degf;
    for (Int i = 0; i != 5; ++i)
    {
        for (Int j = 0; j != 5; ++j)
        {
            for (Int k = 0; k != 5; ++k)
            {
                const auto o = new RigidBody{&_scene, 1.0f, JPH::RefConst{_boxShape.get()}, *_physicsSystem};
                o->translate({i - 2.0f, j + 4.0f, k - 2.0f});
                o->syncPose();
                new ColoredDrawable{
                    *o, _boxInstanceData,
                    Color3::fromHsv({hue += 137.5_degf, 0.75f, 0.9f}),
                    Matrix4::scaling(Vector3{0.5f}), _drawables
                };
            }
        }
    }

    /* Loop at 60 Hz max */
    setSwapInterval(1);
    setMinimalLoopPeriod(16.0_msec);
    _timeline.start();
}

JoltExample::~JoltExample()
{
    _scene.children().clear();

    _physicsSystem = nullptr;

    _groundShape.release();
    _boxShape.release();
    _sphereShape.release();

    _tmpAllocator.release();

    if (JPH::Factory::sInstance != nullptr)
    {
        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
    }
}

void JoltExample::drawEvent() {
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color|GL::FramebufferClear::Depth);

    /* Step Jolt simulation first */
    _physicsSystem->Update(_timeline.previousFrameDuration(), 5, _tmpAllocator.get(), &_jobSystem);

    /* Housekeeping: sync physics and remove far-away objects */
    for(Object3D* obj = _scene.children().first(); obj; ) {
        Object3D* next = obj->nextSibling();

        /* Sync Jolt transform to the scene graph */
        if(auto* rigidBody = dynamic_cast<RigidBody*>(obj)) {
            rigidBody->syncFromPhysics();
        }

        if(obj->transformation().translation().dot() > 100 * 100)
            delete obj;

        obj = next;
    }

    if (_drawCubes)
    {
        /* Populate instance data with transformations and colors */
        arrayClear(_boxInstanceData);
        arrayClear(_sphereInstanceData);
        _camera->draw(_drawables);

        _shader.setProjectionMatrix(_camera->projectionMatrix());

        /* Upload instance data to the GPU (orphaning the previous buffer
        contents) and draw all cubes in one call, and all spheres (if any)
        in another call */
        _boxInstanceBuffer.setData(_boxInstanceData, GL::BufferUsage::DynamicDraw);
        _box.setInstanceCount(_boxInstanceData.size());
        _shader.draw(_box);

        _sphereInstanceBuffer.setData(_sphereInstanceData, GL::BufferUsage::DynamicDraw);
        _sphere.setInstanceCount(_sphereInstanceData.size());
        _shader.draw(_sphere);
    }

    /* Debug draw. If drawing on top of cubes,
     * avoid flickering by setting the depth function to <= instead of just <.
     */
    if (_drawDebug && _debugDraw && _physicsSystem) {
        if (_drawCubes)
            GL::Renderer::setDepthFunction(GL::Renderer::DepthFunction::LessOrEqual);

        _debugDraw->setTransformationProjectionMatrix(
            _camera->projectionMatrix() * _camera->cameraMatrix()
        );

        _physicsSystem->DrawBodies(_debugDraw->drawSettings(), _debugDraw.get());

        _debugDraw->flush();

        if (_drawCubes)
            GL::Renderer::setDepthFunction(GL::Renderer::DepthFunction::Less);
    }

     swapBuffers();
     _timeline.nextFrame();
     redraw();
}

void JoltExample::keyPressEvent(KeyEvent& event) {
     /* Movement */
     if(event.key() == Key::Down) {
         _cameraObject->rotateX(5.0_degf);
     } else if(event.key() == Key::Up) {
         _cameraObject->rotateX(-5.0_degf);
     } else if(event.key() == Key::Left) {
         _cameraRig->rotateY(-5.0_degf);
     } else if(event.key() == Key::Right) {
         _cameraRig->rotateY(5.0_degf);

     /* Toggling draw modes */
     } else if(event.key() == Key::D) {
         if(_drawCubes && _drawDebug) {
             _drawDebug = false;
         } else if(_drawCubes && !_drawDebug) {
             _drawCubes = false;
             _drawDebug = true;
         } else if(!_drawCubes && _drawDebug) {
             _drawCubes = true;
             _drawDebug = true;
         }

     /* What to shoot? */
     } else if(event.key() == Key::S) {
         _shootBox ^= true;
     } else if(event.key() == Key::Esc) {
         exit();
     } else return;

     event.setAccepted();
}

void JoltExample::pointerPressEvent(PointerEvent& event) {
     /* Shoot an object on click */
     if(!event.isPrimary() ||
        !(event.pointer() & (Pointer::MouseLeft|Pointer::Finger)))
         return;

     /* First, scale the position from being relative to window size to being
        relative to framebuffer size as those two can be different on HiDPI
        systems */
     const Vector2 position = event.position()*Vector2{framebufferSize()}/Vector2{windowSize()};
     const Vector2 clickPoint = Vector2::yScale(-1.0f)
        * (position/Vector2{framebufferSize()} - Vector2{0.5f})
        * _camera->projectionSize();
     const Vector3 direction = (_cameraObject->absoluteTransformation().rotationScaling()*Vector3{clickPoint, -1.0f}).normalized();

    /* Shooting */
    const Float shootMass = _shootBox ? 1.0f : 5.0f;
    const JPH::RefConst<JPH::Shape>& shootShape = _shootBox ?
        JPH::RefConst{_boxShape.get()} : JPH::RefConst{_sphereShape.get()};
    auto* object = new RigidBody{&_scene, shootMass, shootShape, *_physicsSystem};
    object->translate(_cameraObject->absoluteTransformation().translation());
    object->syncPose();

    /* Create visual */
    new ColoredDrawable{*object,
        _shootBox ? _boxInstanceData : _sphereInstanceData,
        _shootBox ? 0x880000_rgbf : 0x220000_rgbf,
        Matrix4::scaling(Vector3{_shootBox ? 0.5f : 0.25f}), _drawables};

    /* Shoot it */
    const Vector3 velocityMV = direction * 25.0f;
    const JPH::Vec3 velocityJ = Math::Implementation::VectorConverter<3, Float, JPH::Vec3>::to(velocityMV);
    object->bodyInterface().SetLinearVelocity(object->bodyID(), velocityJ);

    event.setAccepted();
}

}}

MAGNUM_APPLICATION_MAIN(Magnum::Examples::JoltExample)
