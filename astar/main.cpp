#include "prec.h"

using namespace DirectX;

class AStarApp : public wxApp
{
public:
    bool OnInit() override;

private:
    wxFrame* m_pAStarFrame;
};

class AStarFrameUi : public wxFrame
{
public:
    AStarFrameUi(wxWindow* win, wxWindowID id, const wxString& title)
     : wxFrame(win, id, title) {
        
    }
};


namespace astar
{
    struct Node;
    struct Link
    {
        float depth;
        Node* a;
        Node* b;
    };

    struct Node
    {
        float x;
        float y;
    };

    template<typename T>
    struct Ref
    {
        Ref(T& t) : ref{ t } {}

        bool operator == (const T& rhs) const
        {
            return ref == rhs;
        }

        bool operator == (const Ref& rhs) const
        {
            return ref == rhs.ref;
        }

        bool operator != (const T& rhs) const
        {
            return ref != rhs;
        }

        bool operator != (const Ref& rhs) const
        {
            return ref != rhs.ref;
        }

        T* operator->() { return &ref; }

        operator T& () const
        {
            return ref;
        }

        T& ref;
    };


    struct NodeIdSet
    {
        struct Seek
        {
            Seek* prev;
            Seek* next;
            size_t budgetIndex;
        };

        struct Node : Seek
        {
            size_t key;
        };

        struct Iter
        {
            const Seek* seek;
            Iter& operator ++ ()
            {
                seek = seek->next;
                return *this;
            }

            Iter operator ++ (int)
            {
                Iter tmp{ seek };
                seek = seek->next;
                return tmp;
            }

            size_t operator*() const
            {
                return static_cast<const Node*>(seek)->key;
            }

            bool operator==(const Iter& it) const { return seek == it.seek; }

            bool operator!=(const Iter& it) const { return seek != it.seek; }
        };

        NodeIdSet()
            : dummyHead()
            , dummyTail()
            , pool()
            , idleNode()
            , budget()
        {
            dummyHead.next = &dummyTail;
            dummyTail.prev = &dummyHead;
            dummyTail.budgetIndex = std::numeric_limits<size_t>::max();
            for (auto& it : budget)
            {
                it = &dummyTail;
            }

            Node* next = nullptr;
            for (auto& it : pool)
            {
                it.next = next;
                next = &it;
            }

            idleNode = next;
        }

        size_t count(size_t value) const
        {
            return find(value) != end() ? 1 : 0;
        }

        Iter erase(Iter iter)
        {
            if (iter == end())
                return iter;

            Node* item = const_cast<Node*>(static_cast<const Node*>(iter.seek));
            Seek* prev = item->prev;
            Seek* next = item->next;
            prev->next = next;
            next->prev = prev;
            item->next = idleNode;
            idleNode = static_cast<Node*>(item);
            if (budget[item->budgetIndex] == item)
            {
                budget[item->budgetIndex] = item->next;
            }

            return Iter{ next };
        }

        void erase(size_t key)
        {
            const size_t budgetIndex = key % 97;
            Seek* it = budget[budgetIndex];
            Seek* item = nullptr;
            while (it->budgetIndex == budgetIndex)
            {
                auto node = static_cast<Node*>(it);
                if (node->key == key) {
                    item = node;
                    break;
                }

                it = it->next;
            }

            if (item != nullptr)
            {
                Seek* prev = item->prev;
                Seek* next = item->next;
                prev->next = next;
                next->prev = prev;
                item->next = idleNode;
                idleNode = static_cast<Node*>(item);
                if (budget[budgetIndex] == item)
                {
                    budget[budgetIndex] = item->next;
                }
            }
        }

        void insert(size_t key)
        {
            const size_t budgetIndex = key % 97;
            Seek* it = budget[budgetIndex];
            while (it->budgetIndex == budgetIndex)
            {
                auto node = static_cast<Node*>(it);
                if (node->key == key) {
                    // 삽입에 실패했다.
                    return;
                }

                it = it->next;
            }


            Node* newitem = idleNode;
            if (newitem == nullptr)
                return;

            idleNode = static_cast<Node*>(newitem->next);
            newitem->key = key;
            newitem->budgetIndex = budgetIndex;

            Seek* prev = it->prev;
            newitem->prev = prev;
            prev->next = newitem;
            newitem->next = it;
            it->prev = newitem;
            if (budget[budgetIndex]->budgetIndex != budgetIndex)
            {
                budget[budgetIndex] = newitem;
            }
        }

        Iter find(size_t key) const
        {
            size_t budgetIndex = key % 97;
            Seek* it = budget[budgetIndex];
            while (it->budgetIndex == budgetIndex)
            {
                Node& node = static_cast<Node&>(*it);
                if (node.key == key) {
                    return Iter{ &node };
                }

                it = it->next;
            }

            return end();
        }

        Iter begin() const { return Iter{ dummyHead.next }; }

        Iter end() const { return Iter{ &dummyTail }; }

        Seek dummyHead;
        Seek dummyTail;
        Seek* budget[97];
        Node* idleNode;
        Node pool[4096];
    };

    struct GenericPathFinder
    {
        struct NavigatedNode
        {
            size_t nodeId;
            NavigatedNode* prevNode;
            float g;
            float h;
            bool operator == (size_t rhs) const
            {
                return nodeId == rhs;
            }

            bool operator != (size_t rhs) const
            {
                return nodeId != rhs;
            }

            bool operator == (const NavigatedNode& rhs) const
            {
                return nodeId == rhs.nodeId;
            }

            bool operator != (const NavigatedNode& rhs) const
            {
                return nodeId != rhs.nodeId;
            }
        };

        struct NavigatedNodeRef
        {
            NavigatedNodeRef(NavigatedNode& t) : ref{ t } {}

            bool operator == (const NavigatedNode& rhs) const
            {
                return ref == rhs;
            }

            bool operator == (const NavigatedNodeRef& rhs) const
            {
                return ref == rhs.ref;
            }

            bool operator != (const NavigatedNode& rhs) const
            {
                return ref != rhs;
            }

            bool operator != (const NavigatedNodeRef& rhs) const
            {
                return ref != rhs.ref;
            }

            NavigatedNode* operator->() { return &ref; }

            operator NavigatedNode& () const
            {
                return ref;
            }

            NavigatedNode& ref;
        };

        using NodeId = size_t;
        struct Link
        {
            NodeId toNodeId;
            NodeId fromNodeId;
        };

        GenericPathFinder()
            : m_latestNodeId{ 0 }
        {

        }

        ~GenericPathFinder()
        {
            for (auto it : m_linkList)
            {
                delete it;
            }
        }

        NodeId AllocNodeId()
        {
            NodeId newNodeId = m_latestNodeId += 1;
            m_nodeSet.insert(newNodeId);
            return newNodeId;
        }

        NodeId AllocNodeId(NodeId id)
        {
            if (m_nodeSet.count(id) != 0)
                return 0;

            m_nodeSet.insert(id);
            m_latestNodeId = std::max(m_latestNodeId, id);
            return id;
        }

        Link* MakeLink(NodeId toNodeId, NodeId fromNodeId)
        {
            if (m_nodeSet.count(toNodeId) == 0 || m_nodeSet.count(fromNodeId) == 0)
                return nullptr;

            for (auto it : m_linkList)
            {
                if (it->toNodeId != toNodeId || it->fromNodeId != fromNodeId)
                    continue;

                return nullptr;
            }

            auto link = new Link{};
            link->toNodeId = toNodeId;
            link->fromNodeId = fromNodeId;

            m_linkList.push_back(link);
            return link;
        }

        void FreeNode(NodeId nodeId)
        {
            auto itSet = m_nodeSet.find(nodeId);
            if (itSet == m_nodeSet.end())
                return;

            m_nodeSet.erase(itSet);
            auto it = m_linkList.begin();
            while (it != m_linkList.end())
            {
                auto link = *it;
                if (link->toNodeId != nodeId && link->fromNodeId != nodeId)
                {
                    ++it;
                    continue;
                }

                it = m_linkList.erase(it);
                delete link;
            }
        }

        void RemoveLink(Link* link)
        {
            auto it = std::find(m_linkList.begin(), m_linkList.end(), link);
            if (it == m_linkList.end())
                return;

            delete link;
            m_linkList.erase(it);
        }

        std::vector<NavigatedNodeRef>
            CollectLinkedNode(NavigatedNode& srcNode, std::vector<NavigatedNode>& navNode)
        {
            std::vector<NavigatedNodeRef> ret;
            for (auto it : m_linkList)
            {
                if (srcNode != it->fromNodeId)
                    continue;

                ret.push_back(*std::find(navNode.begin(), navNode.end(), it->toNodeId));
            }

            return ret;
        }

        size_t m_latestNodeId;
        NodeIdSet m_nodeSet;
        std::vector<Link*> m_linkList;
    };

    template<typename TFuncterG, typename TFuncterH>
    class TGenericPathFinder: public GenericPathFinder
    {
    public:
        template<typename ArgFuncterG, typename ArgFuncterH>
        TGenericPathFinder(ArgFuncterG&& g, ArgFuncterH&& h)
            : m_functerG{std::forward<ArgFuncterG>(g)}
            , m_functerH{std::forward<ArgFuncterH>(h)}
        {

        }

        bool Navigate(NodeId fromNodeId, NodeId toNodeId, std::vector<NodeId>* result)
        {
            if (fromNodeId == toNodeId)
                return true;

            std::vector<NavigatedNode> nodeList;
            std::list<NavigatedNodeRef> navigatedNodeList;
            static constexpr float initial_max_g = std::numeric_limits<float>::max();
            for (auto nodeId : m_nodeSet)
            {
                NavigatedNode node{};
                node.nodeId = nodeId;
                node.h = m_functerH(nodeId, toNodeId);
                node.g = initial_max_g;
                node.prevNode = nullptr;
                nodeList.push_back(node);
            }

            NavigatedNodeRef navNodeFrom = *std::find(nodeList.begin(), nodeList.end(), fromNodeId);
            NavigatedNodeRef navNodeTo = *std::find(nodeList.begin(), nodeList.end(), toNodeId);
            navNodeFrom->g = 0.f;
            navigatedNodeList.push_back(navNodeFrom);
            do {
                if (navigatedNodeList.empty())
                    return false;

                auto sel = navigatedNodeList.front();
                navigatedNodeList.pop_front();
                if (sel == navNodeTo)
                    break;

                auto nearNodeList = CollectLinkedNode(sel, nodeList);
                for (auto node : nearNodeList)
                {
                    if (node->prevNode == &sel.ref)
                        continue;

                    float g = m_functerG(sel->nodeId, node->nodeId);
                    if (node->g <= g + sel->g)
                        continue;

                    node->g = g + sel->g;
                    node->prevNode = &sel.ref;
                    auto it = std::find(navigatedNodeList.begin(), navigatedNodeList.end(), node);
                    if (it != navigatedNodeList.end())
                    {
                        // 기존 아이템 삭제
                        it = navigatedNodeList.erase(it);
                    }

                    auto end = navigatedNodeList.end();
                    it = navigatedNodeList.begin();

                    for (; it != end; ++it)
                    {
                        auto& rhs = *it;
                        if (node->g + node->h >= rhs->g + rhs->h)
                            continue;

                        navigatedNodeList.insert(it, node);
                        break;
                    }

                    if (it == end)
                    {
                        navigatedNodeList.push_back(node);
                    }
                }
            } while (true);

            result->clear();
            {
                NavigatedNode* it = &navNodeTo.ref;
                while (it != nullptr)
                {
                    result->push_back(it->nodeId);
                    it = it->prevNode;
                }

                std::reverse(result->begin(), result->end());
            }

            return true;
        }

    private:
        TFuncterG m_functerG;
        TFuncterH m_functerH;
    };

    template<typename G, typename H>
    TGenericPathFinder<G, H> CreateGenericPathFinder(G&& g, H&& h)
    {
        return TGenericPathFinder<G, H>{std::forward<G>(g), std::forward<h>(h)};
    }

    class Map
    {
    public:
        struct NavigatedNode
        {
            Node* node;
            NavigatedNode* prevNode;
            float g;
            float h;
            bool operator == (Node* rhs) const
            {
                return node == rhs;
            }

            operator Node* () const
            {
                return node;
            }
        };

        ~Map()
        {
            for (auto it : m_nodeList)
            {
                delete it;
            }

            for (auto it : m_linkList)
            {
                delete it;
            }
        }

        Node* AddNode(float x, float y)
        {
            auto ptr = new Node{};
            ptr->x = x;
            ptr->y = y;
            m_nodeList.push_back(ptr);
            return ptr;
        }

        void UpdateLength()
        {
            for (auto it : m_linkList)
            {
                it->depth = Length(it->a, it->b);
            }
        }

        Link* MakeLink(Node* node1, Node* node2)
        {
            for (auto it : m_linkList)
            {
                if (it->a != node1 && it->b != node1)
                    continue;

                if (it->a != node2 && it->b != node2)
                    continue;

                return nullptr;
            }

            auto link = new Link{};
            link->a = node1;
            link->b = node2;
            link->depth = Length(node1, node2);

            m_linkList.push_back(link);
            return link;
        }

        void RemoveLink(Link* link)
        {

        }
        
        void RemoveNode(Node* node)
        {

        }

        bool Navigate(Node* from, Node* to, std::vector<Node*>* result)
        {
            if (from == to)
                return true;

            std::vector<NavigatedNode> nodeList;
            std::list<Ref<NavigatedNode>> navigatedNodeList;

            for (auto it : m_nodeList)
            {
                NavigatedNode node{};
                node.node = it;
                node.h = Length(it, to);
                node.g = std::numeric_limits<float>::max();
                node.prevNode = nullptr;
                nodeList.push_back(node);
            }

            Ref<NavigatedNode> navNodeFrom = *std::find(nodeList.begin(), nodeList.end(), from);
            Ref<NavigatedNode> navNodeTo = *std::find(nodeList.begin(), nodeList.end(), to);
            navNodeFrom->g = 0.f;
            navigatedNodeList.push_back(navNodeFrom);
            do {
                if (navigatedNodeList.empty())
                    return false;

                auto sel = navigatedNodeList.front();
                navigatedNodeList.pop_front();
                if (sel == navNodeTo)
                    break;

                auto nearNodeList = CollectLinkedNode(sel, nodeList);
                for (auto node : nearNodeList)
                {
                    if (node->prevNode == &sel.ref)
                        continue;

                    float g = Length(node->node, sel->node);
                    if (node->g <= g + sel->g)
                        continue;

                    node->g = g + sel->g;
                    node->prevNode = &sel.ref;
                    auto it = std::find(navigatedNodeList.begin(), navigatedNodeList.end(), node);
                    if (it != navigatedNodeList.end())
                    {
                        // 기존 아이템 삭제
                        it = navigatedNodeList.erase(it);
                    }

                    auto end = navigatedNodeList.end();
                    it = navigatedNodeList.begin();
                    
                    for (; it != end; ++it)
                    {
                        auto& rhs = *it;
                        if (node->g + node->h >= rhs->g + rhs->h)
                            continue;

                        navigatedNodeList.insert(it, node);
                        break;
                    }

                    if (it == end)
                    {
                        navigatedNodeList.push_back(node);
                    }
                }
            } while (true);
            
            result->clear();
            {
                NavigatedNode* it = &navNodeTo.ref;
                while (it != nullptr)
                {
                    result->push_back(it->node);
                    it = it->prevNode;
                }

                std::reverse(result->begin(), result->end());
            }

            return true;
        }

    private:
        static float Length(Node* node1, Node* node2)
        {
            float dx = node1->x - node2->x;
            float dy = node1->y - node2->y;
            return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        }

        std::vector<Ref<NavigatedNode>> CollectLinkedNode(Ref<NavigatedNode> node, std::vector<NavigatedNode>& navNode)
        {
            std::vector<Ref<NavigatedNode>> ret;
            for (auto it : m_linkList)
            {
                if (it->a == node.ref.node)
                {
                    ret.push_back(*std::find(navNode.begin(), navNode.end(), it->b));
                }
                else if (it->b == node.ref.node)
                {
                    ret.push_back(*std::find(navNode.begin(), navNode.end(), it->a));
                }
            }

            return ret;
        }

    private:
        std::vector<Node*> m_nodeList;
        std::vector<Link*> m_linkList;
    };

}

enum {
    ID_ADD_NODE = wxID_HIGHEST + 1,
    ID_ADD_LINK,
    ID_MOVE_NODE,
    ID_FIND_PATH
};

class AStarFrame : public AStarFrameUi
{
public:
    AStarFrame(wxWindow* win, wxWindowID id, const wxString& title)
        : AStarFrameUi(win, id, title)
        , m_selectedToolId{ ID_MOVE_NODE } {
        Bind(wxEVT_CONTEXT_MENU, [this](wxContextMenuEvent& evt)
            {
                wxMenu menu;
                menu.Append(ID_ADD_NODE, wxS("노드 추가"));
                menu.AppendRadioItem(ID_ADD_LINK, wxS("노드 연결"));
                menu.AppendRadioItem(ID_MOVE_NODE, wxS("노드 이동"));
                menu.AppendRadioItem(ID_FIND_PATH, wxS("길 찾기"));
                menu.Check(m_selectedToolId, true);
            PopupMenu(&menu);
        });

        Bind(wxEVT_MENU, [this](wxCommandEvent& evt)
        {
            auto size = GetClientSize();
            size /= 2;
            wxPoint pt{ size.x, size.y };
            auto node = m_map.AddNode(pt.x, pt.y);
            m_nodeList.push_back(node);
            Refresh();
            m_selectedToolId = ID_MOVE_NODE;
        }, ID_ADD_NODE);

        Bind(wxEVT_MENU, [this](wxCommandEvent& evt)
            {
                m_selectedNode = nullptr;
                m_selectedToolId = ID_ADD_LINK;
            }, ID_ADD_LINK);

        Bind(wxEVT_MENU, [this](wxCommandEvent& evt)
            {
                m_selectedNode = nullptr;
                m_selectedToolId = ID_MOVE_NODE;
            }, ID_MOVE_NODE);

        Bind(wxEVT_MENU, [this](wxCommandEvent& evt)
            {
                m_selectedNode = nullptr;
                m_selectedToolId = ID_FIND_PATH;
            }, ID_FIND_PATH);

        Bind(wxEVT_LEFT_DOWN, [this](wxMouseEvent& evt)
        {
            if (m_selectedToolId == ID_MOVE_NODE)
            {
                Refresh();
                m_basePosition = evt.GetPosition();
                m_selectedNode = GetHitNode(m_basePosition);
                if (m_selectedNode != nullptr)
                {
                    m_oldNodePosition.x = m_selectedNode->x;
                    m_oldNodePosition.y = m_selectedNode->y;
                }
            }
            else if (m_selectedToolId == ID_ADD_LINK)
            {
                Refresh();
                auto selectedNode = GetHitNode(evt.GetPosition());
                if (m_selectedNode == nullptr)
                {
                    m_selectedNode = selectedNode;
                    return;
                }
                
                if (selectedNode == nullptr)
                {
                    m_selectedNode = nullptr;
                    return;
                }

                auto link = m_map.MakeLink(selectedNode, m_selectedNode);
                m_selectedNode = nullptr;
                if (link != nullptr)
                {
                    m_linkList.push_back(link);
                }
            }
            else if(m_selectedToolId == ID_FIND_PATH)
            {
                Refresh();
                auto selectedNode = GetHitNode(evt.GetPosition());
                if (m_selectedNode == nullptr)
                {
                    m_pathNodeList.clear();
                    m_selectedNode = selectedNode;
                    return;
                }

                if (selectedNode == nullptr)
                {
                    m_pathNodeList.clear();
                    m_selectedNode = nullptr;
                    return;
                }

                m_map.UpdateLength();
                m_map.Navigate(m_selectedNode, selectedNode, &m_pathNodeList);
                m_selectedNode = nullptr;
            }
        });

        Bind(wxEVT_MOTION, [this](wxMouseEvent& evt)
        {
                if (m_selectedToolId != ID_MOVE_NODE)
                    return;
                
                if (m_selectedNode == nullptr)
                    return;

                Refresh();
                wxPoint newPos = m_oldNodePosition;
                newPos += evt.GetPosition() - m_basePosition;
                m_selectedNode->x = newPos.x;
                m_selectedNode->y = newPos.y;
        });

        Bind(wxEVT_LEFT_UP, [this](wxMouseEvent& evt)
        {
            if (m_selectedToolId == ID_MOVE_NODE && m_selectedNode != nullptr)
            {
                m_selectedNode = nullptr;
                Refresh();
            }
        });

        Bind(wxEVT_PAINT, [this](wxPaintEvent& evt)
            {
                wxPaintDC dc{ this };
                wxBrush whiteBrush{ *wxWHITE_BRUSH };
                wxBrush blackBrush{ *wxBLACK_BRUSH };
                for (auto it : m_linkList)
                {
                    dc.DrawLine(it->a->x, it->a->y, it->b->x, it->b->y);
                }

                dc.SetBrush(whiteBrush);
                for (auto it : m_nodeList)
                {
                    if (it == m_selectedNode)
                    {
                        dc.SetBrush(blackBrush);
                    }

                    dc.DrawEllipse(wxPoint(it->x - 10, it->y - 10), wxSize{ 21, 21 });

                    if (it == m_selectedNode)
                    {
                        dc.SetBrush(whiteBrush);
                    }
                }

                dc.SetBrush(*wxRED_BRUSH);
                for (auto it : m_pathNodeList)
                {
                    dc.DrawEllipse(wxPoint(it->x - 10, it->y - 10), wxSize{ 21, 21 });
                }

                if (!m_pathNodeList.empty() && m_selectedNode != nullptr)
                {
                    dc.SetBrush(blackBrush);
                    dc.DrawEllipse(wxPoint(m_selectedNode->x - 10, m_selectedNode->y - 10), wxSize{ 21, 21 });
                }
            });
    }

    astar::Node* GetHitNode(const wxPoint& pt)
    {
        auto it = m_nodeList.rbegin();
        for (; it != m_nodeList.rend(); ++it)
        {
            auto node = *it;
            float dx = node->x - pt.x;
            float dy = node->y - pt.y;
            if (dx * dx + dy * dy < 441)
            {
                return node;
            }
        }

        return nullptr;
    }

private:
    astar::Map m_map;
    std::vector<astar::Node*> m_nodeList;
    std::vector<astar::Link*> m_linkList;
    std::vector<astar::Node*> m_pathNodeList;
    wxPoint m_basePosition;
    wxPoint m_oldNodePosition;
    astar::Node* m_selectedNode;
    wxWindowID m_selectedToolId;
};

class AStarFrame2 : public AStarFrameUi
{
public:
    struct FuncterG
    {
        AStarFrame2* frame;
        float operator()(astar::GenericPathFinder::NodeId src, astar::GenericPathFinder::NodeId dst) const
        {
            auto srcPt = frame->m_nodeTable[src];
            auto dstPt = frame->m_nodeTable[dst];
            float dx = dstPt.x - srcPt.x;
            float dy = dstPt.y - srcPt.y;
            return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        }
    };

    struct FunctionH
    {
        AStarFrame2* frame;
        float operator()(astar::GenericPathFinder::NodeId src, astar::GenericPathFinder::NodeId finalDest) const
        {
            auto srcPt = frame->m_nodeTable[src];
            auto dstPt = frame->m_nodeTable[finalDest];
            float dx = dstPt.x - srcPt.x;
            float dy = dstPt.y - srcPt.y;
            return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        }
    };

    AStarFrame2(wxWindow* win, wxWindowID id, const wxString& title)
        : AStarFrameUi(win, id, title)
        , m_pathFinder{ new astar::TGenericPathFinder<FuncterG, FunctionH>(FuncterG{this}, FunctionH{this}) }
        , m_selectedToolId{ ID_MOVE_NODE }
        , m_selectedNode{ nullptr } {
        auto toolBar = CreateToolBar(wxTB_DEFAULT_STYLE | wxTB_TEXT);
        toolBar->AddTool(wxID_OPEN, wxGetStockLabel(wxID_OPEN), wxArtProvider::GetBitmapBundle(wxART_FILE_OPEN, wxART_TOOLBAR), wxGetStockHelpString(wxID_OPEN));
        toolBar->AddTool(wxID_SAVE, wxGetStockLabel(wxID_SAVE), wxArtProvider::GetBitmapBundle(wxART_FILE_SAVE, wxART_TOOLBAR), wxGetStockHelpString(wxID_SAVE));
        toolBar->Realize();
        m_drawPanel = new wxPanel{ this };
        m_drawPanel->Bind(wxEVT_CONTEXT_MENU, [this](wxContextMenuEvent& evt)
            {
                wxMenu menu;
                menu.Append(ID_ADD_NODE, wxS("노드 추가"));
                menu.AppendRadioItem(ID_ADD_LINK, wxS("노드 연결"));
                menu.AppendRadioItem(ID_MOVE_NODE, wxS("노드 이동"));
                menu.AppendRadioItem(ID_FIND_PATH, wxS("길 찾기"));
                menu.Check(m_selectedToolId, true);
                m_drawPanel->PopupMenu(&menu);
            });

        Bind(wxEVT_MENU, [this](wxCommandEvent& evt)
            {
                auto size = GetClientSize();
                size /= 2;
                wxPoint pt{ size.x, size.y };
                auto nodeId = m_pathFinder->AllocNodeId();
                m_nodeTable[nodeId] = pt;
                m_drawPanel->Refresh();
                m_selectedToolId = ID_MOVE_NODE;
            }, ID_ADD_NODE);

        Bind(wxEVT_MENU, [this](wxCommandEvent& evt)
            {
                m_selectedNode = nullptr;
                m_selectedToolId = ID_ADD_LINK;
            }, ID_ADD_LINK);

        Bind(wxEVT_MENU, [this](wxCommandEvent& evt)
            {
                m_selectedNode = nullptr;
                m_selectedToolId = ID_MOVE_NODE;
            }, ID_MOVE_NODE);

        Bind(wxEVT_MENU, [this](wxCommandEvent& evt)
            {
                m_selectedNode = nullptr;
                m_selectedToolId = ID_FIND_PATH;
            }, ID_FIND_PATH);

        m_drawPanel->Bind(wxEVT_LEFT_DOWN, [this](wxMouseEvent& evt)
            {
                if (m_selectedToolId == ID_MOVE_NODE)
                {
                    m_drawPanel->Refresh();
                    m_basePosition = evt.GetPosition();
                    m_selectedNode = GetHitNode(m_basePosition);
                    if (m_selectedNode != nullptr)
                    {
                        m_oldNodePosition = m_selectedNode->second;
                    }
                }
                else if (m_selectedToolId == ID_ADD_LINK)
                {
                    m_drawPanel->Refresh();
                    auto selectedNode = GetHitNode(evt.GetPosition());
                    if (m_selectedNode == nullptr)
                    {
                        m_selectedNode = selectedNode;
                        return;
                    }

                    if (selectedNode == nullptr)
                    {
                        m_selectedNode = nullptr;
                        return;
                    }

                    auto link = m_pathFinder->MakeLink(selectedNode->first, m_selectedNode->first);
                    m_selectedNode = nullptr;
                    if (link != nullptr)
                    {
                        m_linkList.emplace_back(link);
                    }
                }
                else if (m_selectedToolId == ID_FIND_PATH)
                {
                    Refresh();
                    auto selectedNode = GetHitNode(evt.GetPosition());
                    if (m_selectedNode == nullptr)
                    {
                        m_pathNodeList.clear();
                        m_selectedNode = selectedNode;
                        return;
                    }

                    if (selectedNode == nullptr)
                    {
                        m_pathNodeList.clear();
                        m_selectedNode = nullptr;
                        return;
                    }

                    m_pathFinder->Navigate(m_selectedNode->first, selectedNode->first, &m_pathNodeList);
                    m_selectedNode = nullptr;
                }
            });

        m_drawPanel->Bind(wxEVT_MOTION, [this](wxMouseEvent& evt)
            {
                if (m_selectedToolId != ID_MOVE_NODE)
                    return;

                if (m_selectedNode == nullptr)
                    return;

                m_drawPanel->Refresh();
                wxPoint newPos = m_oldNodePosition;
                newPos += evt.GetPosition() - m_basePosition;
                m_selectedNode->second = newPos;
            });

        m_drawPanel->Bind(wxEVT_LEFT_UP, [this](wxMouseEvent& evt)
            {
                if (m_selectedToolId == ID_MOVE_NODE && m_selectedNode != nullptr)
                {
                    m_selectedNode = nullptr;
                    m_drawPanel->Refresh();
                }
            });

        m_drawPanel->Bind(wxEVT_PAINT, [this](wxPaintEvent& evt)
            {
                wxPaintDC dc{ m_drawPanel };
                wxBrush whiteBrush{ *wxWHITE_BRUSH };
                wxBrush blackBrush{ *wxBLACK_BRUSH };


                wxPoint array[3]{
                    {-11, 0},
                    {-26, 15},
                    {-26, -15}
                };

                for (auto link : m_linkList)
                {
                    auto& pt1 = m_nodeTable[link->toNodeId];
                    auto& pt2 = m_nodeTable[link->fromNodeId];
                    auto vPtTo = XMVectorSet(pt1.x, pt1.y, 0.f, 0.f);
                    auto vPtFr = XMVectorSet(pt2.x, pt2.y, 0.f, 0.f);
                    auto vArrow = XMVector2Normalize(vPtTo - vPtFr);
                    auto vTop = XMVector2Orthogonal(vArrow);
                    XMFLOAT2 s;
                    XMFLOAT2 s2;
                    XMStoreFloat2(&s, vArrow);
                    XMStoreFloat2(&s2, vTop);
                    wxAffineMatrix2D mat{};
                    mat.Set(wxMatrix2D{ s.x, s.y, s2.x, s2.y }, wxPoint2DDouble{ static_cast<double>(pt1.x), static_cast<double>(pt1.y) });

                    dc.DrawLine(pt1, pt2);

                    dc.SetTransformMatrix(mat);
                    dc.DrawPolygon(3, array);
                    dc.ResetTransformMatrix();
                }

                dc.SetBrush(whiteBrush);
                for (auto& it : m_nodeTable)
                {
                    if (&it == m_selectedNode)
                    {
                        dc.SetBrush(blackBrush);
                    }

                    dc.DrawEllipse(it.second - wxPoint(10, 10), wxSize{ 21, 21 });

                    if (&it == m_selectedNode)
                    {
                        dc.SetBrush(whiteBrush);
                    }
                }

                dc.SetBrush(*wxRED_BRUSH);
                for (auto it : m_pathNodeList)
                {
                    auto& pt = m_nodeTable[it];
                    dc.DrawEllipse(pt - wxPoint(10, 10), wxSize{ 21, 21 });
                }

                if (!m_pathNodeList.empty() && m_selectedNode != nullptr)
                {
                    dc.SetBrush(blackBrush);
                    dc.DrawEllipse(m_selectedNode->second - wxPoint(10, 10), wxSize{ 21, 21 });
                }
            });

        Bind(wxEVT_TOOL, [this](wxCommandEvent& evt)
            {
              
                wxFileDialog dialog{ this, wxFileSelectorPromptStr, {}, {}, wxS("xml file|*.xml"), wxFD_OPEN };
                auto id = dialog.ShowModal();
                if (id == wxID_CANCEL)
                    return;

                wxXmlDocument doc;
                doc.Load(dialog.GetPath());
                auto root = doc.GetRoot();
                if (root->GetName() != wxS("map"))
                    return;


                auto pathFinder = std::make_unique<astar::TGenericPathFinder<FuncterG, FunctionH>>(FuncterG {this}, FunctionH{ this });
                std::map<astar::GenericPathFinder::NodeId, wxPoint> nodeTable;
                std::vector<astar::GenericPathFinder::Link*> linkList;
                wxXmlNode* it;
                it = root->GetChildren();
                while (it != nullptr)
                {
                    wxXmlNode* child = it;
                    it = it->GetNext();

                    if (child->GetName() != wxS("node"))
                        continue;

                    long idLong;
                    long x;
                    long y;
                    wxString attr = child->GetAttribute(wxS("id"));
                    if (!attr.ToLong(&idLong))
                        return;

                    attr = child->GetAttribute(wxS("x"));
                    if (!attr.ToLong(&x))
                        return;

                    attr = child->GetAttribute(wxS("y"));
                    if (!attr.ToLong(&y))
                        return;

                    nodeTable.emplace(idLong, wxPoint{ x, y });
                    pathFinder->AllocNodeId(idLong);
                }

                it = root->GetChildren();
                while (it != nullptr)
                {
                    wxXmlNode* child = it;
                    it = it->GetNext();

                    if (child->GetName() != wxS("link"))
                        continue;

                    long fromNodeId;
                    long toNodeId;
                    auto attr = child->GetAttribute(wxS("from"));
                    if (!attr.ToLong(&fromNodeId))
                        return;

                    attr = child->GetAttribute(wxS("to"));
                    if (!attr.ToLong(&toNodeId))
                        return;

                    auto link = pathFinder->MakeLink(toNodeId, fromNodeId);
                    if (link == nullptr)
                        return;

                    linkList.push_back(link);
                }

                m_pathFinder = std::move(pathFinder);
                m_nodeTable.swap(nodeTable);
                m_linkList.swap(linkList);
                m_pathNodeList.clear();
                Refresh();
            }, wxID_OPEN);

        Bind(wxEVT_TOOL, [this](wxCommandEvent& evt)
            {

                wxFileDialog dialog{ this, wxFileSelectorPromptStr, {}, {}, wxS("xml file|*.xml"), wxFD_SAVE };
                auto id = dialog.ShowModal();
                if (id == wxID_CANCEL)
                    return;

                auto filePath = dialog.GetPath();
                wxFile file;
                if (!file.Open(filePath, wxFile::write))
                    return;

                wxFileOutputStream os{ file };
                wxXmlDocument doc;
                wxXmlNode* rootNode = new wxXmlNode{ wxXML_ELEMENT_NODE, wxS("map") };
                for (auto& pair : m_nodeTable)
                {
                    wxXmlNode* mapNode = new wxXmlNode{ wxXML_ELEMENT_NODE, wxS("node") };
                    mapNode->AddAttribute(wxS("id"), wxString() << pair.first);
                    mapNode->AddAttribute(wxS("x"), wxString() << pair.second.x);
                    mapNode->AddAttribute(wxS("y"), wxString() << pair.second.y);
                    rootNode->AddChild(mapNode);
                }

                for (auto& link : m_linkList)
                {
                    wxXmlNode* linkNode;
                    linkNode = new wxXmlNode{ wxXML_ELEMENT_NODE, wxS("link") };
                    linkNode->AddAttribute(wxS("from"), wxString() << link->fromNodeId);
                    linkNode->AddAttribute(wxS("to"), wxString() << link->toNodeId);
                    rootNode->AddChild(linkNode);
                }

                doc.SetRoot(rootNode);
                doc.SetFileEncoding(wxS("utf-8"));
                doc.SetVersion(wxS("1.0"));
                doc.Save(os);

            }, wxID_SAVE);
    }

    std::pair<const astar::GenericPathFinder::NodeId, wxPoint>*
        GetHitNode(const wxPoint& pt)
    {
        auto it = m_nodeTable.rbegin();
        for (; it != m_nodeTable.rend(); ++it)
        {
            auto& node = it->second;
            auto dx = node.x - pt.x;
            auto dy = node.y - pt.y;
            if (dx * dx + dy * dy < 441)
            {
                return std::addressof(*it);
            }
        }

        return nullptr;
    }

private:
    wxPanel* m_drawPanel;
    std::unique_ptr<astar::TGenericPathFinder<FuncterG, FunctionH>> m_pathFinder;
    std::map<astar::GenericPathFinder::NodeId, wxPoint> m_nodeTable;
    std::pair<const astar::GenericPathFinder::NodeId, wxPoint>* m_selectedNode;
    std::vector<astar::GenericPathFinder::Link*> m_linkList;
    std::vector<astar::GenericPathFinder::NodeId> m_pathNodeList;
    wxPoint m_basePosition;
    wxPoint m_oldNodePosition;
    wxWindowID m_selectedToolId;
};

bool AStarApp::OnInit()
{
    if (!wxApp::OnInit())
    {
        return false;
    }

    m_pAStarFrame = new AStarFrame2{ nullptr, wxID_ANY, wxS("A* test") };
    m_pAStarFrame->Show();

    return true;
}

wxIMPLEMENT_APP(AStarApp);
