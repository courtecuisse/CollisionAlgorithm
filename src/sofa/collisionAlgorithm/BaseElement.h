#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa::collisionAlgorithm {

class BaseGeometry;

class BaseElement {
public:
    typedef std::shared_ptr<BaseElement> SPtr;

    class ElementIterator {
        typedef BaseProximity::Index Index;
    public:

        ///defines a unique pointer iterator of baseElements
        class SPtr : public std::shared_ptr<ElementIterator> {
        public:
            SPtr(ElementIterator * ptr) : std::shared_ptr<ElementIterator>(ptr) {}

            //we take the geometry as parameter for std::iterator compatibility i.e. it != m_geo->end();
            bool operator != (const BaseGeometry * ) { return ! this->get()->end(); }

            //we take the geometry as parameter for std::iterator compatibility i.e. it != m_geo->end();
            bool operator== (const BaseGeometry * ) { return this->get()->end(); }

            void operator++() { this->get()->next(); }

            void operator++(int) { this->get()->next(); }

            const BaseElement::SPtr operator* () { return this->get()->element(); }

            const BaseElement::SPtr operator* () const { return this->get()->element(); }
        };

        //This is necessary to have the correct behavior with the delete !
        virtual ~ElementIterator() = default;

        virtual bool end() const = 0;

        virtual void next() = 0;

        virtual BaseElement::SPtr element() = 0;

        virtual const BaseElement::SPtr element() const = 0;

        ///returns a new EmptyIterator
        static ElementIterator::SPtr empty() {
            class EmptyIterator : public ElementIterator {
            public:
                bool end() const override { return true; }

                void next() override {}

                BaseElement::SPtr element() override { return NULL; }

                const BaseElement::SPtr element() const override { return NULL; }
            };

            return SPtr(new EmptyIterator());
        }
    };

    typedef ElementIterator::SPtr Iterator;

    virtual void update() = 0;
//    virtual BaseProximity::SPtr createProximity(CONTROL_POINT id = CONTROL_DEFAULT) const = 0;

//    virtual Index elementSize() const = 0;

//    virtual Index id() const = 0;
};


//template<class ElementType>
//class TBaseElement : public BaseElement {
//public:

//    static BaseElement::SPtr create(const ElementType & cont) {
//        return BaseElement::SPtr(new TBaseElement(cont));
//    }
//};

}
