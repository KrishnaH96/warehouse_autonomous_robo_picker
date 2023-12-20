#include <gtest/gtest.h>
#include <memory>
#include <queue>


class Context;

class State
{   
    public:
        /**
         * @brief Construct a new State object
         * 
         * @param contextPtr 
         */
        State(Context&  contextPtr) 
                    : contextPtr_(contextPtr){}
        virtual void Execute() {
            
        }

    protected:
        Context& contextPtr_;

};

class Context
{
    private:
        std::queue<std::shared_ptr<State>> runQueue;

    public:
        Context();
        void setState(std::shared_ptr<State> _state);
        void execute();
};


Context::Context() {

}

void Context::setState(std::shared_ptr<State> _state) {
    // runQueue.pop();
    runQueue.push(_state);
    execute();
}

void Context::execute() {
        std::shared_ptr<State> s = runQueue.front();
        s->Execute();
}





class StateMock: public State
{
    /* data */
  public:
    StateMock(Context& _context);
    int count = 0;
    void Execute();
};

StateMock::StateMock(Context& _context) : State(_context) {

}

void StateMock::Execute() {
      count++;
}


TEST(context_test, execute_verification) {
  std::shared_ptr<Context> context = std::make_shared<Context>();
  std::shared_ptr<StateMock> mockState = std::make_shared<StateMock>(*context);

  context->setState(mockState);

  EXPECT_EQ(mockState->count, 1);
}

// TEST(dummy_test, this_should_pass_too) {
//   Context
//   EXPECT_EQ(function1(3), 103);
// }

// TEST(dummy_test, this_will_fail) {
//   EXPECT_EQ(function2(32), function1(32));
// }
